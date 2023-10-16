import rclpy
from rclpy.node import Node
from kinova_driver_msg.msg import Command, JointFeedback, KinovaFeedback
from user_interface.kinova_controller import Controller
from threading import Thread


class Controller_Node(Node):
    def __init__(self):
        super().__init__("kinova_controller_node")
        self.publisher = self.create_publisher(Command, "/kinova_driver/command", 10)
        self.subscription = self.create_subscription(
            KinovaFeedback, "/kinova_driver/feedback", self.callback, 10
        )

        self.start_position = None
        self.controller = Controller()
        spin_thread = Thread(target=self.start_spin_loop)
        spin_thread.start()
        self.controller.start_render_loop()

    def callback(self, msg: KinovaFeedback):
        for joint in self.controller.joints:
            joint_feedback: JointFeedback = getattr(msg, joint.name)
            joint.position = joint_feedback.position
            joint.speed = joint_feedback.speed
            joint.voltage = joint_feedback.voltage
            joint.current = joint_feedback.current
        self.controller.update_feedback()

    def start_spin_loop(self):
        rclpy.spin(self)


def main(args=None) -> None:
    """Main."""
    rclpy.init(args=args)
    Controller_Node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
