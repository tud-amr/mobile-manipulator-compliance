import os
import rclpy
from rclpy.node import Node
from threading import Thread

from user_interface_msg.msg import Ufdbk, Ustate, Ucmd

from compliant_control.interface.user_interface import UserInterface
from compliant_control.mujoco.visualization import Visualization


class UserInterfaceNode(Node):
    """A node that starts the user interface."""

    def __init__(self) -> None:
        super().__init__("user_interface_node")
        self.declare_parameter("visualize", True)
        self.create_subscription(Ufdbk, "/feedback", self.feedback, 10)
        self.create_subscription(Ustate, "/state", self.state, 10)
        self.publisher = self.create_publisher(Ucmd, "/command", 10)

        self.interface = UserInterface(self.command)
        self.interface.create_ui()

        spin_thread = Thread(target=self.start_spin_loop)
        spin_thread.start()

        self.visualize = (
            self.get_parameter("visualize").get_parameter_value().bool_value
        )
        if self.visualize:
            self.visualization = Visualization()
            visualize_thread = Thread(target=self.visualization.start)
            visualize_thread.start()

        self.interface.start()

    def command(self, command: str, args: list = None) -> None:
        """Send a command to the control interface."""
        self.interface.update_state("waiting")
        msg = Ucmd()
        msg.command = command
        if args is not None:
            msg.args = args
        self.publisher.publish(msg)

    def feedback(self, msg: Ufdbk) -> None:
        """Process the feedback."""
        if len(msg.kinova_pos) > 0:
            self.kinova_feedback(msg)
        if len(msg.dingo_pos) > 0:
            self.dingo_feedback(msg)

    def state(self, msg: Ustate) -> None:
        """Update the state."""
        self.interface.state.mode = msg.mode
        self.interface.state.comp_grav = msg.comp_grav
        self.interface.state.comp_fric = msg.comp_fric
        self.interface.state.imp_joint = msg.imp_joint
        self.interface.state.imp_cart = msg.imp_cart
        for n, joint in enumerate(self.interface.joints):
            joint.active = msg.joint_active[n]
            joint.mode = msg.joint_mode[n]
        self.interface.update_state()

    def kinova_feedback(self, msg: Ufdbk) -> None:
        """Process the Kinova feedback."""
        for joint in self.interface.joints:
            joint.pos = msg.kinova_pos[joint.index]
            joint.vel = msg.kinova_vel[joint.index]
            joint.eff = msg.kinova_tor[joint.index]
        self.interface.rates.kin = msg.kinova_rate
        self.interface.update_bars("Kinova")
        if self.visualize:
            self.visualization.set_qpos_value("Kinova", "position", msg.kinova_pos)

    def dingo_feedback(self, msg: Ufdbk) -> None:
        """Process the Dingo feedback."""
        for wheel in self.interface.wheels:
            last_position = wheel.pos
            wheel.encoder_position = msg.dingo_pos[wheel.index]
            wheel.vel = wheel.pos - last_position
            wheel.eff = msg.dingo_tor[wheel.index]
        self.interface.rates.din = msg.dingo_rate
        self.interface.update_bars("Dingo")
        if self.visualize:
            self.visualization.set_qpos_value("Dingo", "position", msg.dingo_pos)

    def start_spin_loop(self) -> None:
        """Start node spinning."""
        rclpy.spin(self)


def main(args: any = None) -> None:
    """Main."""
    rclpy.init(args=args)
    UserInterfaceNode()
    rclpy.shutdown()
    os._exit(0)


if __name__ == "__main__":
    main()
