import rclpy
import os
import time
from rclpy.node import Node
from threading import Thread

from compliant_control.control.state import State
from compliant_control.mujoco.simulation import Simulation
from compliant_control.kinova.kortex_client import KortexClient
from compliant_control.kinova.kortex_client_simulation import KortexClientSimulation

from user_interface_msg.msg import Ufdbk, Ucmd, Ustate

PUBLISH_RATE = 100


class ControlInterfaceNode(Node):
    """A node that starts the control interface of the robot."""

    def __init__(self) -> None:
        super().__init__("control_interface_node")
        self.pub_fdbk = self.create_publisher(Ufdbk, "/feedback", 10)
        self.pub_state = self.create_publisher(Ustate, "/state", 10)
        self.create_subscription(Ucmd, "/command", self.handle_input, 10)
        simulation = True

        self.state = State()

        if simulation:
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
        self.kinova = KortexClient(self.state)

    def start_simulation(self) -> None:
        """Start the simulation."""
        simulation = Simulation()
        self.kinova = KortexClientSimulation(self.state, simulation)
        self.kinova.log = self.get_logger().warn
        self.start_threads()
        simulation.start()

    def start_publish_loop(self) -> None:
        """Start a loop that publishes the feedback."""
        while True:
            self.publish_feedback()
            time.sleep(1 / PUBLISH_RATE)

    def publish_feedback(self) -> None:
        """Publish feedback."""
        feedback = Ufdbk()
        feedback.kinova_pos = list(self.state.feedback.q)
        feedback.kinova_vel = list(self.state.feedback.dq)
        feedback.kinova_tor = list(self.state.feedback.c)
        feedback.kinova_rate = self.kinova.rate
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
            case "Stop LLC Task":
                self.kinova.disconnect_LLC()
            case "Clear Faults":
                self.kinova.clear_faults()
            case _ if cmd in [str(n) for n in range(self.kinova.actuator_count)]:
                self.kinova.toggle_active(int(cmd))
            case _:
                print(f"Service call {cmd} is unknown.")
        self.publish_state()

    def publish_state(self) -> None:
        """Publish the state of the robot."""
        state = Ustate()
        state.mode = self.kinova.mode
        state.joint_active = self.kinova.joint_active
        state.joint_mode = self.kinova.get_control_modes()
        self.pub_state.publish(state)

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
