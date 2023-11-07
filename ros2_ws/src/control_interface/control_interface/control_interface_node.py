import rclpy
import os
import time
import signal
from rclpy.node import Node
from threading import Thread

from compliant_control.control.state import State
from compliant_control.mujoco.simulation import Simulation

from compliant_control.dingo.dingo_driver import DingoDriver
from compliant_control.dingo.dingo_driver_simulation import DingoDriverSimulation

from compliant_control.kinova.kortex_client import KortexClient
from compliant_control.kinova.kortex_client_simulation import KortexClientSimulation
from compliant_control.kinova.utilities import DeviceConnection

from compliant_control.control.calibration import Calibration

from user_interface_msg.msg import Ufdbk, Ucmd, Ustate

PUBLISH_RATE = 100


class ControlInterfaceNode(Node):
    """A node that starts the control interface of the robot."""

    def __init__(self) -> None:
        super().__init__("control_interface_node")
        self.declare_parameter("simulate", True)
        self.simulate = self.get_parameter("simulate").get_parameter_value().bool_value

        self.pub_fdbk = self.create_publisher(Ufdbk, "/feedback", 10)
        self.pub_state = self.create_publisher(Ustate, "/state", 10)
        self.create_subscription(Ucmd, "/command", self.handle_input, 10)

        self.state = State()
        self.state.simulation = self.simulate

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
        self.dingo.log = self.get_logger().info
        self.start_threads()
        with DeviceConnection.createTcpConnection() as router, DeviceConnection.createUdpConnection() as real_time_router:
            self.kinova = KortexClient(
                self.state, router=router, real_time_router=real_time_router
            )
            self.kinova.log = self.get_logger().info
            signal.signal(signal.SIGINT, self.kinova.stop)
            self.kinova.start()

    def start_simulation(self) -> None:
        """Start the simulation."""
        self.simulation = Simulation(self.state)
        self.dingo = DingoDriverSimulation(self.state, self.simulation)
        self.kinova = KortexClientSimulation(self.state, self.simulation)
        self.kinova.log = self.get_logger().info
        self.kinova.start_in_new_thread()
        self.calibration = Calibration(self.state, self.kinova)
        self.calibration.log = self.get_logger().info
        self.start_threads()
        self.simulation.start()

    def start_publish_loop(self) -> None:
        """Start a loop that publishes the feedback."""
        while not hasattr(self, "kinova"):
            time.sleep(0.5)
        self.get_logger().info("READY!")
        while True:
            self.publish_feedback()
            time.sleep(1 / PUBLISH_RATE)

    def publish_feedback(self) -> None:
        """Publish feedback."""
        feedback = Ufdbk()
        feedback.kinova_pos = list(self.state.kinova_feedback.q)
        feedback.kinova_vel = list(self.state.kinova_feedback.dq)
        feedback.kinova_tor = list(self.state.kinova_feedback.c)
        feedback.kinova_rate = self.kinova.rate
        feedback.dingo_pos = list(self.state.dingo_feedback.q)
        feedback.dingo_vel = list(self.state.dingo_feedback.dq)
        feedback.dingo_tor = list(self.state.dingo_feedback.c)
        feedback.dingo_rate = self.dingo.rate
        self.pub_fdbk.publish(feedback)

    def handle_input(self, msg: Ucmd) -> None:
        """Handle user input."""
        cmd = msg.command
        match cmd:
            case "Refresh":
                pass
            case "Home":
                self.kinova.home()
            case "Zero":
                self.kinova.zero()
            case "Retract":
                self.kinova.retract()
            case "Pref":
                self.kinova.pref()
            case "Start LLC":
                self.kinova.start_LLC()
            case "Stop LLC":
                self.kinova.stop_LLC()
            case "Start LLC Task":
                self.kinova.connect_LLC()
                self.reset_target()
            case "Stop LLC Task":
                self.kinova.disconnect_LLC()
            case "Clear Faults":
                self.kinova.clear_faults()
            case _ if cmd in [str(n) for n in range(self.kinova.actuator_count)]:
                self.kinova.toggle_active(int(cmd))
            case _ if cmd in ["gravity", "friction", "joint", "cartesian"]:
                self.state.controller.toggle(cmd)
                if cmd in ["joint", "cartesian"]:
                    self.reset_target()
            case "Move Dingo":
                self.state.controller.command_base_direction(msg.args)
            case "Static":
                self.calibration.calibrate_all("static")
            case "Dynamic":
                self.calibration.calibrate_all("dynamic")
            case _:
                print(f"Service call {cmd} is unknown.")
        self.publish_state()

    def publish_state(self) -> None:
        """Publish the state of the robot."""
        state = Ustate()
        state.servoing_mode = self.kinova.mode
        state.comp_grav = self.state.controller.comp_grav
        state.comp_fric = self.state.controller.comp_fric
        state.imp_joint = self.state.controller.imp_joint
        state.imp_cart = self.state.controller.imp_cart

        state.active = self.kinova.joint_active
        state.mode = self.kinova.get_control_modes()
        state.ratio = list(self.state.ratios)
        state.fric_s = self.state.static_frictions
        state.fric_d = self.state.dynamic_frictions

        self.pub_state.publish(state)

    def reset_target(self) -> None:
        """Reset the target."""
        if self.simulate:
            self.simulation.update_target(self.simulation.end_effector)
        self.state.target = self.state.x.copy()

    def keep_alive_loop(self) -> None:
        """Keep alive."""
        while True:
            time.sleep(1)

    def start_spin_loop(self) -> None:
        """Start node spinning."""
        rclpy.spin(self)


def main(args: any = None) -> None:
    """Main."""
    rclpy.init(args=args)
    ControlInterfaceNode()
    rclpy.shutdown()
    os._exit(0)


if __name__ == "__main__":
    main()
