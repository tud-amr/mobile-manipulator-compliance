import rclpy
from rclpy.node import Node
import os
from compliant_control.mujoco.viewer import Viewer
from threading import Thread
from kinova_driver_msg.msg import KinFdbk, KinTar
from simulation_msg.srv import SimSrv
from dingo_driver_msg.msg import DinFdbk


class VisualizationNode(Node):
    """A node that starts the visualization of the robot."""

    def __init__(self) -> None:
        super().__init__("visualization_node")
        self.create_subscription(KinFdbk, "/kinova/fdbk", self.kin_fdbk, 10)
        self.create_subscription(DinFdbk, "/dingo/fdbk", self.din_fdbk, 10)
        self.create_service(SimSrv, "/sim/srv", self.srv)
        self.tar_pub = self.create_publisher(KinTar, "/kinova/tar", 10)
        self.spin_thread = Thread(target=self.start_spin_loop)
        self.vis = Viewer("visualization", self.step_callback)
        self.spin_thread.start()
        self.vis.start()

    def srv(self, request: SimSrv.Request, response: SimSrv.Response) -> None:
        """Execute service call."""
        match request.name:
            case "ToggleAutomove":
                self.vis.toggle_automove_target()
            case "ResetTarget":
                self.vis.update_target(self.vis.end_effector)
        response.success = True
        return response

    def kin_fdbk(self, msg: KinFdbk) -> None:
        """Update the visualization based on the Kinova feedback."""
        self.vis.set_qpos_value("Kinova", "position", msg.joint_pos)

    def din_fdbk(self, msg: DinFdbk) -> None:
        """Update the visualization based on the Dingo feedback."""
        self.vis.set_qpos_value("Dingo", "position", msg.wheel_pos)

    def step_callback(self) -> None:
        """Callback that is called every visualization step."""
        self.publish_target()

    def publish_target(self) -> None:
        """Publish the target."""
        target = KinTar()
        target.target = list(self.vis.relative_target)
        self.tar_pub.publish(target)

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
