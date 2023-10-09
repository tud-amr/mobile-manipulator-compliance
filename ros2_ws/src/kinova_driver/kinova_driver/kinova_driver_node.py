import rclpy
from rclpy.node import Node
from kinova_driver_msg.msg import Feedback, Command
from kinova_driver_msg.srv import HighLevelMove
import subprocess
from kinova import utilities
from kinova.kortex_client import KortexClient


class KinovaDriverNode(Node):
    def __init__(self):
        super().__init__("kinova_driver_node")
        self.publisher = self.create_publisher(
            Command, "/kinova_driver_node/command", 10
        )
        self.subscription = self.create_subscription(
            Feedback, "/kinova_driver/feedback", self.callback, 10
        )
        self.service = self.create_service(
            HighLevelMove, "/kinova/high_level_move", self.high_level_move
        )

        if not self.ip_available():
            print("Kinova arm not found, exiting...")
            return

        with utilities.DeviceConnection.createTcpConnection() as router, utilities.DeviceConnection.createUdpConnection() as real_time_router:
            self.kortex_client = KortexClient(
                router=router, real_time_router=real_time_router
            )
            self.start_spin_loop()

    def high_level_move(
        self, request: HighLevelMove.Request, response: HighLevelMove.Response
    ):
        match request.position:
            case "home":
                self.kortex_client.home()
            case "zero":
                self.kortex_client.zero()
            case "retract":
                self.kortex_client.retract()
            case _:
                print("High level movement is unknown.")

        return response

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
