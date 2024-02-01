from __future__ import annotations
import rospy
import os
import time
import signal
import sys

from threading import Thread
import numpy as np

from compliant_control.control.state import State
from compliant_control.mujoco.simulation import Simulation

from compliant_control.dingo.dingo_driver import DingoDriver
from compliant_control.dingo.dingo_driver_simulation import DingoDriverSimulation

from compliant_control.kinova.kortex_client import KortexClient
from compliant_control.kinova.kortex_client_simulation import KortexClientSimulation
from compliant_control.kinova.utilities import DeviceConnection

from compliant_control.control.calibration import Calibration

from user_interface_msg.msg import Ufdbk, Ucmd, Ustate, Utarget, Record, Data

PUBLISH_RATE = 100
D_MAX_RESET = 0.01


class ControlInterfaceNode:
    """A node that starts the control interface of the robot."""

    def __init__(self, args) -> None:
        rospy.init_node("control_interface_node")
        self.simulate = "--simulate" in args

        self.pub_fdbk = rospy.Publisher("/feedback", Ufdbk, queue_size=10)
        self.pub_state = rospy.Publisher("/state", Ustate, queue_size=10)
        self.pub_record = rospy.Publisher("/record", Record, queue_size=10)
        self.pub_calibration = rospy.Publisher("/calibration", Data, queue_size=10)
        rospy.Subscriber("/command", Ucmd, self.handle_input, queue_size=10)
        rospy.Subscriber("/target", Utarget, self.update_target, queue_size=10)

        self.automove_target = False
        self.state = State(self.simulate)

        if self.simulate:
            self.start_simulation()
        else:
            self.start_robot()

    def start_threads(self) -> None:
        """Start the threads."""
        spin_thread = Thread(target=self.start_spin_loop)
        spin_thread.start()

        publish_thread = Thread(target=self.start_publish_loop)
        publish_thread.start()

    def start_robot(self) -> None:
        """Start the robot."""
        self.dingo = DingoDriver(self.state)
        self.dingo.log = rospy.loginfo
        self.start_threads()
        with DeviceConnection.createTcpConnection() as router, DeviceConnection.createUdpConnection() as real_time_router:
            self.kinova = KortexClient(
                self.state, router=router, real_time_router=real_time_router
            )
            self.kinova.log = rospy.loginfo
            self.calibration = Calibration(
                self.state, self.kinova, self.publish_calibration
            )
            self.calibration.log = rospy.loginfo
            signal.signal(signal.SIGINT, self.kinova.stop)
            self.kinova.start()

    def start_simulation(self) -> None:
        """Start the simulation."""
        self.simulation = Simulation(self.state)
        self.dingo = DingoDriverSimulation(self.state, self.simulation)
        self.kinova = KortexClientSimulation(self.state, self.simulation)
        self.kinova.log = rospy.loginfo
        self.kinova.start_in_new_thread()
        self.calibration = Calibration(
            self.state, self.kinova, self.publish_calibration
        )
        self.calibration.log = rospy.loginfo
        self.start_threads()
        self.simulation.start()

    def start_publish_loop(self) -> None:
        """Start a loop that publishes the feedback."""
        while not hasattr(self, "kinova"):
            time.sleep(0.5)
        rospy.loginfo("READY!")
        while True:
            self.publish_feedback()
            self.publish_record()
            time.sleep(1 / PUBLISH_RATE)

    def publish_feedback(self) -> None:
        """Publish feedback."""
        feedback = Ufdbk()
        feedback.kinova_pos = list(self.state.kinova_feedback.q)
        feedback.kinova_vel = list(self.state.kinova_feedback.dq)
        feedback.kinova_tor = list(self.state.kinova_feedback.c)
        feedback.kinova_rate = self.kinova.rate_counter.rate
        feedback.dingo_pos = list(self.state.dingo_feedback.q)
        feedback.dingo_vel = list(self.state.dingo_feedback.dq)
        feedback.dingo_tor = list(self.state.dingo_feedback.c)
        feedback.dingo_rate = self.dingo.rate_counter.rate
        feedback.controller_rate = self.state.controller.rate_counter.rate
        self.pub_fdbk.publish(feedback)

    def publish_record(self) -> None:
        """Publish data to record."""
        msg = Record()
        msg.pos_x = list(self.state.x)
        msg.pos_q = list(self.state.kinova_feedback.q)
        msg.pos_b = list(self.state.pos_base)
        msg.quat_b = list(self.state.quat_base)
        msg.relative_target = list(self.state.target)
        msg.absolute_target = list(self.state.absolute_target)
        self.pub_record.publish(msg)

    def publish_calibration(self, data: np.ndarray) -> None:
        """Publish the data."""
        msg = Data()
        msg.data = list(data)
        self.pub_calibration.publish(msg)

    def handle_input(self, msg: Ucmd) -> None:
        """Handle user input."""
        cmd = msg.command
        if cmd == "Refresh":
            pass
        elif cmd == "Home":
            self.kinova.home()
        elif cmd == "Zero":
            self.kinova.zero()
        elif cmd == "Retract":
            self.kinova.retract()
        elif cmd == "Pref":
            self.kinova.pref()
        elif cmd == "Start HLT":
            if self.target_is_at_end_effector():
                self.kinova.start_HLT()
        elif cmd == "Stop HLT":
            self.kinova.stop_HLT()
        elif cmd == "Start LLC":
            self.kinova.start_LLC()
        elif cmd == "Stop LLC":
            self.kinova.stop_LLC()
        elif cmd == "Start LLC Task":
            if self.target_is_at_end_effector():
                self.kinova.connect_LLC()
        elif cmd == "Stop LLC Task":
            self.kinova.disconnect_LLC()
        elif cmd == "Clear":
            self.kinova.clear_faults()
        elif cmd == "Open":
            self.kinova._move_gripper(close=False)
        elif cmd == "Close":
            self.kinova._move_gripper(close=True)
        elif cmd == "Automove":
            self.toggle_automove_target()
        elif cmd == "Reset":
            if self.simulate:
                self.simulation.reset_target()
        elif cmd in [str(n) for n in range(self.kinova.actuator_count)]:
            self.kinova.toggle_active(int(cmd))
        elif cmd in ["grav", "fric", "arm", "null", "base"]:
            if cmd == "arm" and not self.target_is_at_end_effector():
                pass
            else:
                self.state.controller.toggle(cmd)
        elif cmd == "Move Dingo":
            self.state.controller.reset_base_command()
            self.state.controller.command_base_direction(msg.args, 0.6)
        elif cmd == "Calibrate":
            self.calibration.calibrate_all()
        else:
            print(f"Service call {cmd} is unknown.")
        self.publish_state()

    def publish_state(self) -> None:
        """Publish the state of the robot."""
        state = Ustate()
        state.servoing_mode = self.kinova.mode
        state.comp_grav = self.state.controller.comp_grav
        state.comp_fric = self.state.controller.comp_fric
        state.imp_arm = self.state.controller.imp_arm
        state.imp_null = self.state.controller.imp_null
        state.imp_base = self.state.controller.imp_base

        state.active = self.kinova.joint_active
        state.mode = self.kinova.get_control_modes()
        state.ratio = list(self.state.ratios)
        state.friction = list(self.state.frictions)
        state.automove_target = self.automove_target

        self.pub_state.publish(state)

    def target_is_at_end_effector(self) -> None:
        """Reset the target."""
        if self.simulate:
            self.simulation.reset_target()
            self.state.target = self.state.x
        return np.linalg.norm(self.state.target - self.state.x) < D_MAX_RESET

    def update_target(self, msg: Utarget) -> None:
        """Update the target."""
        self.state.target = np.array(msg.relative_target)
        self.state.absolute_target = np.array(msg.absolute_target)
        self.state.pos_base = np.array(msg.pos_b)
        self.state.quat_base = np.array(msg.quat_b)

    def toggle_automove_target(self) -> None:
        """Toggle automove of target."""
        if self.simulate:
            self.simulation.target_mover.toggle()
            self.automove_target = self.simulation.target_mover.active
        else:
            self.automove_target = not self.automove_target

    def step_callback(self) -> None:
        """Called every simulation step."""
        self.state.target = self.simulation.relative_target

    def start_spin_loop(self) -> None:
        """Start node spinning."""
        rospy.spin()


def main(args: any = None) -> None:
    """Main."""
    args = rospy.myargv(argv=sys.argv)
    ControlInterfaceNode(args)
    os._exit(0)


if __name__ == "__main__":
    main()
