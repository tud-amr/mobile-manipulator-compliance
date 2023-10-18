import rclpy
import os
from rclpy.node import Node
from threading import Thread
from kinova.mujoco_viewer import MujocoViewer
from kinova_driver_msg.msg import KinovaFeedback, JointFeedback
from mujoco_viewer_msg.msg import MujocoFeedback
from std_msgs.msg import MultiArrayDimension


class MujocoViewerNode(Node):
    """A node that starts the mujoco visualization."""

    def __init__(self) -> None:
        super().__init__("mujoco_visualization_node")
        self.create_subscription(KinovaFeedback, "/kinova/feedback", self.callback, 10)
        self.publisher = self.create_publisher(MujocoFeedback, "/mujoco/feedback", 10)

        self.mujoco_viewer = MujocoViewer()
        self.mujoco_viewer.callback = self.publish
        spin_thread = Thread(target=self.start_spin_loop)
        spin_thread.start()
        self.mujoco_viewer.start_visualization()

    def callback(self, msg: KinovaFeedback) -> None:
        """Callback on kinova data."""
        for n in range(self.mujoco_viewer.model.njnt):
            joint_feedback: JointFeedback = getattr(msg, f"joint{n}")
            self.mujoco_viewer.data.qpos[n] = joint_feedback.position
            self.mujoco_viewer.data.qvel[n] = joint_feedback.speed

    def publish(self) -> None:
        """Publish mujoco feedback."""
        feedback = MujocoFeedback()
        data = self.mujoco_viewer.data.xfrc_applied
        x = MultiArrayDimension()
        x.size = data.shape[0]
        y = MultiArrayDimension()
        y.size = data.shape[1]
        feedback.perturbations.layout.dim = [x, y]
        feedback.perturbations.data = list(self.mujoco_viewer.data.xfrc_applied.flatten())
        self.publisher.publish(feedback)

    def start_spin_loop(self) -> None:
        """Start the ros spin loop."""
        rclpy.spin(self)


def main(args: any = None) -> None:
    """Main."""
    rclpy.init(args=args)
    MujocoViewerNode()
    rclpy.shutdown()
    os._exit(0)


if __name__ == "__main__":
    main()
