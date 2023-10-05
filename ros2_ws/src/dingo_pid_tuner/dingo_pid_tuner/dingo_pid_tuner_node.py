import rclpy
from rclpy.node import Node
from dingo_driver_msg.msg import Feedback
from dingo_driver_msg.srv import SetGain
from user_interface.dingo_pid_tuner import Tuner
from threading import Thread
import time


class PID_Tuner_Interface_Node(Node):
    def __init__(self):
        super().__init__("pid_tuner_interface")
        self.subscription = self.create_subscription(
            Feedback, "/dingo_driver/feedback", self.callback, 10
        )
        self.client = self.create_client(SetGain, "set_gain")

        self.tuner = Tuner(self.set_gain)
        spin_thread = Thread(target=self.start_spin_loop)
        spin_thread.start()
        self.tuner.start_render_loop()

    def set_gain(self, gain: str, value: float):
        request = SetGain.Request()
        request.gain = gain
        request.value = float(value)
        future = self.client.call_async(request)
        while not future.done():
            time.sleep(0.1)
        if future.result():
            setattr(self.tuner, gain, value)

    def callback(self, msg: Feedback):
        self.tuner.update_data(msg.x, msg.y_feedback)

    def start_spin_loop(self):
        rclpy.spin(self)


def main(args=None) -> None:
    """Main."""
    rclpy.init(args=args)
    PID_Tuner_Interface_Node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
