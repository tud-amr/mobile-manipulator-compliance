import rclpy
from rclpy.node import Node
import os
from compliant_control.mujoco.viewer import Viewer
from threading import Thread
from kinova_driver_msg.msg import KinFdbk
from dingo_driver_msg.msg import DinFdbk


class VisualizationNode(Node):
    """A node that starts the visualization of the robot."""

    def __init__(self) -> None:
        super().__init__("visualization_node")
        self.create_subscription(KinFdbk, "/kinova/fdbk", self.kin_fdbk, 10)
        self.create_subscription(DinFdbk, "/dingo/fdbk", self.din_fdbk, 10)
        self.spin_thread = Thread(target=self.start_spin_loop)
        self.vis = Viewer("visualization")
        self.spin_thread.start()
        self.vis.start()

    def kin_fdbk(self, msg: KinFdbk) -> None:
        """Update the visualization based on the Kinova feedback."""
        self.vis.set_qpos_value("Kinova", "position", msg.joint_pos)

    def din_fdbk(self, msg: DinFdbk) -> None:
        """Update the visualization based on the Dingo feedback."""
        self.vis.set_qpos_value("Dingo", "position", msg.wheel_pos)

    def start_spin_loop(self) -> None:
        """Start the ros spin loop."""
        rclpy.spin(self)


def main(args: any = None) -> None:
    """Main."""
    rclpy.init(args=args)
    VisualizationNode()
    rclpy.shutdown()
    os._exit(0)


if __name__ == "__main__":
    main()
