import os
import rclpy
from rclpy.node import Node
from kinova_driver_msg.msg import JointFeedback, KinovaFeedback, JointState, KinovaState
from dingo_driver_msg.msg import WheelFeedback, DingoFeedback
from kinova_driver_msg.srv import Service
from mujoco_viewer_msg.srv import ToggleAutomove
from compliant_control.interface.user_interface import UserInterface
from threading import Thread
import time


class UserInterfaceNode(Node):
    """A node that starts the user interface."""

    def __init__(self) -> None:
        super().__init__("user_interface_node")
        self.create_subscription(
            KinovaFeedback, "/kinova/feedback", self.kinova_feedback, 10
        )
        self.create_subscription(
            DingoFeedback, "/dingo/feedback", self.dingo_feedback, 10
        )
        self.create_subscription(KinovaState, "/kinova/state", self.kinova_state, 10)
        self.kinova_client = self.create_client(Service, "/kinova/service")
        self.mujoco_client = self.create_client(ToggleAutomove, "/mujoco/automove")

        self.start_position = None
        self.interface = UserInterface(self.callback)
        self.interface.toggle_automove_target = self.toggle_mujoco_automove
        spin_thread = Thread(target=self.start_spin_loop)
        spin_thread.start()
        initialize_thread = Thread(target=self.callback, args=["Initialize"])
        initialize_thread.start()
        self.interface.start_render_loop()

    def toggle_mujoco_automove(self) -> None:
        """Toggle target automove of mujoco."""
        future = self.mujoco_client.call_async(ToggleAutomove.Request())
        while not future.done():
            time.sleep(0.1)
        self.interface.automove_target = future.result().state
        self.interface.update_control()

    def callback(self, name: str) -> None:
        """Connect the interface with service calls."""
        self.interface.mode = "waiting"
        self.interface.update_control()
        request = Service.Request()
        request.name = name
        future = self.kinova_client.call_async(request)
        while not future.done():
            time.sleep(0.1)
        self.interface.mode = future.result().mode
        self.interface.update_control()

    def kinova_feedback(self, msg: KinovaFeedback) -> None:
        """Update the Kinova feedback."""
        for joint in self.interface.joints:
            joint_feedback: JointFeedback = getattr(msg, joint.name)
            joint.position = joint_feedback.position
            joint.speed = joint_feedback.speed
            joint.current = joint_feedback.current
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

    def kinova_state(self, msg: KinovaState) -> None:
        """Update the Kinova state."""
        for joint in self.interface.joints:
            joint_state: JointState = getattr(msg, joint.name)
            joint.active = joint_state.active
            joint.mode = joint_state.mode[:3]
            joint.ratio = joint_state.ratio
            joint.fric_d = joint_state.fric_d
            joint.fric_s = joint_state.fric_s
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
