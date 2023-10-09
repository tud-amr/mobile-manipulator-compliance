import rclpy
from rclpy.node import Node
from kinova_driver_msg.msg import Feedback, Command
from kinova_driver_msg.srv import HighLevelMove
from threading import Thread
import subprocess
from kinova import utilities


class KinovaDriverNode(Node):
    def __init__(self):
        super().__init__("kinova_driver_node")
        self.publisher = self.create_publisher(Command, "/kinova_driver_node/command", 10)
        self.subscription = self.create_subscription(
            Feedback, "/kinova_driver/feedback", self.callback, 10
        )
        self.client = self.create_client(HighLevelMove, "set_gain")

        print(f"IP availabe: {str(self.ip_available())}")

        spin_thread = Thread(target=self.start_spin_loop)
        spin_thread.start()

    def callback(self, msg: Feedback):
        pass

    def start_spin_loop(self):
        rclpy.spin(self)

    def ip_available(self) -> bool:
        """Check if robot is available."""
        return (
            subprocess.call(
                "ping -c 1 -W 0.1 " + utilities.DEFAULT_IP,
                shell=True,
                stdout=subprocess.DEVNULL,
            )
            == 0
        )


def main(args=None) -> None:
    """Main."""
    rclpy.init(args=args)
    KinovaDriverNode()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
