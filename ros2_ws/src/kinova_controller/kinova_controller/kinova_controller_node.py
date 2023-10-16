import rclpy
from rclpy.node import Node
from kinova_driver_msg.msg import Command, JointFeedback, KinovaFeedback
from kinova_driver_msg.srv import Service
from user_interface.kinova_controller import Controller
from threading import Thread
import time


class ControlInterfaceNode(Node):
    """A node that starts the control interface."""

    def __init__(self) -> None:
        super().__init__("kinova_controller_node")
        self.publisher = self.create_publisher(Command, "/kinova_driver/command", 10)
        self.subscription = self.create_subscription(
            KinovaFeedback, "/kinova_driver/feedback", self.update_feedback, 10
        )
        self.client = self.create_client(Service, "/kinova/service")

        self.start_position = None
        self.controller = Controller(self.callback)
        spin_thread = Thread(target=self.start_spin_loop)
        spin_thread.start()
        self.controller.start_render_loop()

    def callback(self, name: str) -> None:
        """Connect the interface with service calls."""
        self.controller.mode = "waiting"
        request = Service.Request()
        request.name = name
        future = self.client.call_async(request)
        while not future.done():
            time.sleep(0.1)
        self.controller.mode = future.result().mode

    def update_feedback(self, msg: KinovaFeedback) -> None:
        """Update the feedback."""
        for joint in self.controller.joints:
            joint_feedback: JointFeedback = getattr(msg, joint.name)
            joint.position = joint_feedback.position
            joint.speed = joint_feedback.speed
            joint.voltage = joint_feedback.voltage
            joint.current = joint_feedback.current
        self.controller.update_feedback()

    def start_spin_loop(self) -> None:
        """Start the ros spin loop."""
        rclpy.spin(self)


def main(args: any = None) -> None:
    """Main."""
    rclpy.init(args=args)
    ControlInterfaceNode()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
