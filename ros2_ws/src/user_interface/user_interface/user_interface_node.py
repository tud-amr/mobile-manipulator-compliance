import os
import rclpy
from rclpy.node import Node
from kinova_driver_msg.msg import KinFdbk, KinSts
from dingo_driver_msg.msg import WheelFeedback, DingoFeedback, DingoCommand
from kinova_driver_msg.srv import KinSrv
from simulation_msg.srv import SimSrv
from compliant_control.interface.user_interface import UserInterface
from threading import Thread
import numpy as np


class UserInterfaceNode(Node):
    """A node that starts the user interface."""

    def __init__(self) -> None:
        super().__init__("user_interface_node")
        self.create_subscription(KinFdbk, "/kinova/fdbk", self.kin_fdbk, 10)
        self.create_subscription(
            DingoFeedback, "/dingo/feedback", self.dingo_feedback, 10
        )
        self.create_subscription(KinSts, "/kinova/sts", self.kin_sts, 10)
        self.kinova_client = self.create_client(KinSrv, "/kinova/srv")
        self.sim_client = self.create_client(SimSrv, "/sim/srv")
        self.dingo_pub = self.create_publisher(DingoCommand, "/dingo/command", 10)

        self.interface = UserInterface()
        self.interface.callbacks.buttons = self.callback
        self.interface.callbacks.joystick = self.command_dingo

        self.interface.toggle_automove_target = self.toggle_automove_target
        spin_thread = Thread(target=self.start_spin_loop)
        spin_thread.start()
        initialize_thread = Thread(target=self.callback, args=["Initialize"])
        initialize_thread.start()
        self.interface.start_render_loop()

    def toggle_automove_target(self) -> None:
        """Toggle target automove of mujoco."""
        request = SimSrv.Request()
        request.name = "ToggleAutomove"
        self.sim_client.call(request)

    def command_dingo(self, direction: list) -> None:
        """Send a command to Dingo."""
        m = np.linalg.norm(direction) * 3
        angle = np.arctan2(*direction)

        command = DingoCommand()
        orientations = ["l", "r", "r", "l"]
        n = 0
        for fr in ["f", "r"]:
            for lr in ["l", "r"]:
                orientation = orientations[n]
                torque = self.calculate_torque(angle, orientation) * m
                torque *= 1 if lr == "l" else 1
                setattr(command, fr + lr, torque)
                n += 1
        self.dingo_pub.publish(command)

    def calculate_torque(self, angle: float, orientation: str) -> float:
        """Calculate the torque."""
        torques = [-1, -1, -1, 0, 1, 1, 1, 0, -1]
        angles = np.linspace(-np.pi, np.pi, 9)
        if orientation == "l":
            torques.reverse()
        return np.interp(angle, angles, torques)

    def callback(self, name: str) -> None:
        """Connect the interface with service calls."""
        self.interface.mode = "waiting"
        self.interface.update_control()
        request = KinSrv.Request()
        request.name = name
        self.kinova_client.call_async(request)

    def kin_fdbk(self, msg: KinFdbk) -> None:
        """Update the Kinova feedback."""
        for joint in self.interface.joints:
            joint.position = msg.joint_pos[joint.index]
            joint.speed = msg.joint_vel[joint.index]
            joint.current = msg.joint_tor[joint.index]
        self.interface.update_rate = msg.update_rate
        self.interface.update_kinova_plots()

    def dingo_feedback(self, msg: DingoFeedback) -> None:
        """Update the Dingo feedback."""
        for wheel in self.interface.wheels:
            wheel_feedback: WheelFeedback = getattr(msg, wheel.name)
            last_position = wheel.position
            wheel.encoder_position = wheel_feedback.position
            wheel.speed = wheel.position - last_position
            wheel.power = wheel_feedback.current * wheel_feedback.voltage
        self.interface.update_dingo_plots()

    def kin_sts(self, msg: KinSts) -> None:
        """Update the Kinova state."""
        for joint in self.interface.joints:
            joint.active = msg.joint_active[joint.index]
            joint.mode = msg.joint_mode[joint.index][:3]
            joint.ratio = msg.joint_ratio[joint.index]
            joint.fric_d = msg.joint_fric_d[joint.index]
            joint.fric_s = msg.joint_fric_s[joint.index]
        self.interface.mode = msg.mode
        self.interface.servoing = msg.servoing
        self.interface.compensate_friction = msg.compensate_friction
        self.interface.automove_target = msg.automove_target
        self.interface.update_control()
        self.interface.update_state()

    def start_spin_loop(self) -> None:
        """Start the ros spin loop."""
        rclpy.spin(self)


def main(args: any = None) -> None:
    """Main."""
    rclpy.init(args=args)
    UserInterfaceNode()
    rclpy.shutdown()
    os._exit(0)


if __name__ == "__main__":
    main()
