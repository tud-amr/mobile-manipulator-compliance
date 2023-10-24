import rclpy
import os
import numpy as np
import mujoco
from rclpy.node import Node
from threading import Thread
from compliant_control.kinova.mujoco_viewer import MujocoViewer
from kinova_driver_msg.msg import KinovaFeedback, JointFeedback
from dingo_driver_msg.msg import DingoFeedback
from mujoco_viewer_msg.msg import MujocoFeedback, MujocoCommand
from mujoco_viewer_msg.srv import ToggleAutomove
from std_msgs.msg import MultiArrayDimension

JOINTS = 6


class MujocoViewerNode(Node):
    """A node that starts the mujoco visualization."""

    def __init__(self) -> None:
        super().__init__("mujoco_visualization_node")
        self.create_subscription(KinovaFeedback, "/kinova/feedback", self.kinova_cb, 10)
        self.create_subscription(DingoFeedback, "/dingo/feedback", self.dingo_cb, 10)
        self.create_subscription(MujocoCommand, "/mujoco/command", self.mujoco_cb, 10)
        self.publisher = self.create_publisher(MujocoFeedback, "/mujoco/feedback", 10)
        self.create_service(ToggleAutomove, "/mujoco/automove", self.toggle_automove)

        self.mujoco_viewer = MujocoViewer()
        self.mujoco_viewer.callback = self.publish
        spin_thread = Thread(target=self.start_spin_loop)
        spin_thread.start()
        self.mujoco_viewer.start_visualization()

    def dingo_cb(self, msg: DingoFeedback) -> None:
        """Callback on dingo data."""
        idx = mujoco.mj_name2id(
            self.mujoco_viewer.model, mujoco.mjtObj.mjOBJ_JOINT, "D_J_B"
        )
        idpos = self.mujoco_viewer.model.jnt_qposadr[idx]
        self.mujoco_viewer.data.qpos[idpos] = msg.base_pos_x
        self.mujoco_viewer.data.qpos[idpos + 1] = msg.base_pos_y
        self.mujoco_viewer.data.qpos[idpos + 6] = msg.base_rot_z

    def kinova_cb(self, msg: KinovaFeedback) -> None:
        """Callback on kinova data."""
        for n in range(JOINTS):
            joint_feedback: JointFeedback = getattr(msg, f"joint{n}")
            idx = mujoco.mj_name2id(
                self.mujoco_viewer.model, mujoco.mjtObj.mjOBJ_JOINT, f"K_J{n}"
            )
            idpos = self.mujoco_viewer.model.jnt_qposadr[idx]
            idvel = self.mujoco_viewer.model.jnt_dofadr[idx]
            self.mujoco_viewer.data.qpos[idpos] = joint_feedback.position
            self.mujoco_viewer.data.qvel[idvel] = joint_feedback.speed

    def mujoco_cb(self, msg: MujocoCommand) -> None:
        """Update the target."""
        self.mujoco_viewer.update_target(np.array(msg.target.data))

    def toggle_automove(
        self, _: ToggleAutomove.Request, response: ToggleAutomove.Response
    ) -> ToggleAutomove.Response:
        """Toggle automove of target."""
        self.mujoco_viewer.toggle_automove()
        response.state = self.mujoco_viewer.automove_target
        return response

    def publish(self) -> None:
        """Publish mujoco feedback."""
        feedback = MujocoFeedback()
        data = self.mujoco_viewer.data.xfrc_applied
        x = MultiArrayDimension()
        x.size = data.shape[0]
        y = MultiArrayDimension()
        y.size = data.shape[1]
        feedback.perturbations.layout.dim = [x, y]
        feedback.perturbations.data = list(
            self.mujoco_viewer.data.xfrc_applied.flatten()
        )
        feedback.target.data = list(self.mujoco_viewer.target)
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
