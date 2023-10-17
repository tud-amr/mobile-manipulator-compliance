import os
import rclpy
from rclpy.node import Node
from kinova_driver_msg.msg import (
    Command,
    JointFeedback,
    KinovaFeedback,
    JointState,
    KinovaState,
)
from kinova_driver_msg.srv import Service
from user_interface.kinova_controller import Controller
from threading import Thread
import time


class ControlInterfaceNode(Node):
    """A node that starts the control interface."""

    def __init__(self) -> None:
        super().__init__("kinova_controller_node")
        self.publisher = self.create_publisher(Command, "/kinova_driver/command", 10)
        self.create_subscription(
            KinovaFeedback, "/kinova/feedback", self.update_feedback, 10
        )
        self.create_subscription(KinovaState, "/kinova/state", self.update_state, 10)
        self.client = self.create_client(Service, "/kinova/service")

        self.start_position = None
        self.controller = Controller(self.callback)
        spin_thread = Thread(target=self.start_spin_loop)
        spin_thread.start()
        initialize_thread = Thread(target=self.callback, args=["Initialize"])
        initialize_thread.start()
        self.controller.start_render_loop()

    def callback(self, name: str) -> None:
        """Connect the interface with service calls."""
        self.controller.mode = "waiting"
        self.controller.update_widgets()
        request = Service.Request()
        request.name = name
        future = self.client.call_async(request)
        while not future.done():
            time.sleep(0.1)
        self.controller.mode = future.result().mode
        self.controller.update_widgets()

    def update_feedback(self, msg: KinovaFeedback) -> None:
        """Update the feedback."""
        for joint in self.controller.joints:
            joint_feedback: JointFeedback = getattr(msg, joint.name)
            joint.position = joint_feedback.position
            joint.speed = joint_feedback.speed
            joint.voltage = joint_feedback.voltage
            joint.current = joint_feedback.current
        self.controller.update_rate = msg.update_rate
        self.controller.update_feedback()

    def update_state(self, msg: KinovaState) -> None:
        """Update the state."""
        for joint in self.controller.joints:
            joint_state: JointState = getattr(msg, joint.name)
            joint.active = joint_state.active
            joint.mode = joint_state.mode[:3]
            joint.ratio = joint_state.ratio
            joint.fric_d = joint_state.fric_d
            joint.fric_s = joint_state.fric_s
        self.controller.mode = msg.mode
        self.controller.servoing = msg.servoing
        self.controller.update_widgets()
        self.controller.update_state()

    def start_spin_loop(self) -> None:
        """Start the ros spin loop."""
        rclpy.spin(self)


def main(args: any = None) -> None:
    """Main."""
    rclpy.init(args=args)
    ControlInterfaceNode()
    rclpy.shutdown()
    os._exit(0)


if __name__ == "__main__":
    main()
