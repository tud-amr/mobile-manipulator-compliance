import rclpy
from rclpy.node import Node
from dingo_driver_msg.msg import DingoFeedback, WheelFeedback, Command
from user_interface.dingo_controller import Controller
from threading import Thread


class Controller_Node(Node):
    def __init__(self):
        super().__init__("dingo_controller_node")
        self.publisher = self.create_publisher(Command, "/dingo_driver/command", 10)
        self.subscription = self.create_subscription(
            DingoFeedback, "/dingo_driver/feedback", self.callback, 10
        )

        self.start_position = None
        self.controller = Controller()
        spin_thread = Thread(target=self.start_spin_loop)
        spin_thread.start()
        self.controller.start_render_loop()

    def callback(self, msg: DingoFeedback):
        command = Command()
        for wheel in self.controller.wheels:
            wheel_feedback: WheelFeedback = getattr(msg, wheel.name)
            if wheel.start_position is None:
                wheel.start_position = wheel_feedback.position
            new_position = wheel.start_position - wheel_feedback.position
            if "left" in wheel.name:
                wheel.speed = new_position - wheel.position
            else:
                wheel.speed = wheel.position - new_position
            wheel.position = new_position
            wheel.voltage = wheel_feedback.voltage
            wheel.current = wheel_feedback.current
            setattr(command, wheel.name, wheel.command)
        self.publisher.publish(command)
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
