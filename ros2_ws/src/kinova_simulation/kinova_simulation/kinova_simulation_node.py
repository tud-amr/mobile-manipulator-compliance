import rclpy
import os
from rclpy.node import Node
from kinova_driver_msg.msg import Feedback, Command
from kinova_driver_msg.srv import HighLevelMove
from kinova.kortex_client_simulation import KortexClientSimulation
from threading import Thread


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

        self.kortex_client = KortexClientSimulation()
        spin_thread = Thread(target=self.start_spin_loop)
        spin_thread.start()
        self.kortex_client.mujoco_viewer.start_simulation()

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


def main(args=None) -> None:
    """Main."""
    rclpy.init(args=args)
    KinovaDriverNode()
    rclpy.shutdown()
    os._exit(0)


if __name__ == "__main__":
    main()
