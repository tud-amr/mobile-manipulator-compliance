import rclpy
from rclpy.node import Node
from dingo_driver_msg.msg import Feedback
from user_interface.dingo_controller import Controller
from threading import Thread


class Controller_Node(Node):
    def __init__(self):
        super().__init__("pid_tuner_interface")
        self.subscription = self.create_subscription(
            Feedback, "/dingo_driver/feedback", self.callback, 10
        )

        self.controller = Controller()
        spin_thread = Thread(target=self.start_spin_loop)
        spin_thread.start()
        self.controller.start_render_loop()

    def callback(self, msg: Feedback):
        self.controller.update_data(msg.x, msg.position, msg.current)

    def start_spin_loop(self):
        rclpy.spin(self)


def main(args=None) -> None:
    """Main."""
    rclpy.init(args=args)
    Controller_Node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
