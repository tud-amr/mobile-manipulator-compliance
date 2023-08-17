from typing import TYPE_CHECKING, Literal
import importlib.resources as pkg_resources
import mujoco
import numpy as np
import pinocchio
import kinova.models as models
from user_interface.mujoco_viewer import MujocoViewer

if TYPE_CHECKING:
    from kinova.kortex_client import KortexClient


class State:
    """Contains the state of the robot."""

    def __init__(self, mujoco_viewer: MujocoViewer) -> None:
        self.model = mujoco_viewer.model
        self.data = mujoco_viewer.data
        self.load_robot()

        self.M: np.ndarray
        self.C: np.ndarray
        self.g: np.ndarray
        self.J: np.ndarray
        self.dJ: np.ndarray

    def connect_client(self, client: "KortexClient") -> None:
        """Connect the client."""
        self.client = client
        self.active = [True] * client.actuator_count
        self.q = np.empty(client.actuator_count)
        self.dq = np.empty(client.actuator_count)
        self.current_torque_ratios = (
            {0: 1, 1: 1, 2: 1, 3: 1, 4: 1, 5: 1}
            if client.mock
            else {0: 0, 1: 0.2911, 2: 1.136, 3: 1.727, 4: 1.448, 5: 0}
        )
        self.joint_frictions = (
            {0: 0.9, 1: 0, 2: 0, 3: 0, 4: 0.6, 5: 0}
            if self.client.mock
            else {0: 0.7, 1: 0, 2: 0, 3: 0, 4: 0.3, 5: 0}
        )

    def update(self) -> None:
        """Update the state of the robot."""
        for n in range(self.client.actuator_count):
            self.q[n] = self.client.get_position(n, False)
            self.dq[n] = self.client.get_velocity(n, False)
        if not self.client.simulate:
            self.data.qpos = self.q
            self.data.qvel = self.dq

        self.robot.forwardKinematics(self.q, self.dq)
        self.robot.computeJointJacobians(self.q)
        self.robot.framesForwardKinematics(self.q)

        self.update_x()
        self.update_M()
        self.update_C()
        self.update_g()
        self.update_J()
        self.update_dJ()
        self.update_target()
        self.update_marker("end_effector", self.x)

    def toggle_joint(self, joint: int) -> None:
        """Toggle active state of joint."""
        self.active[joint] = not self.active[joint]

    def get_joint_state(self, joint: int) -> bool:
        """Return whether the given joint is active."""
        return self.active[joint]

    def get_ratio(self, joint: int) -> float:
        """Get the current/torque ratio between the model and the real robot for the given joint."""
        return self.current_torque_ratios[joint]

    def load_robot(self) -> None:
        """Load the pinocchio robot."""
        urdf_package = str(pkg_resources.files(models))
        urdf = urdf_package + "/GEN3-LITE.urdf"
        self.robot = pinocchio.RobotWrapper.BuildFromURDF(urdf, urdf_package)
        frame_id = self.robot.model.getFrameId("GRIPPER_FRAME")
        joint_id = self.robot.model.getJointId("5")
        location = pinocchio.SE3(1)
        location.translation = np.array([0, 0, 0.09])
        frame = pinocchio.Frame(
            "END_EFFECTOR",
            joint_id,
            frame_id,
            location,
            pinocchio.OP_FRAME,
        )
        self.robot.model.addFrame(frame)
        self.robot.data = pinocchio.createDatas(self.robot.model)[0]

    def update_x(self) -> np.ndarray:
        """Get the location of the end effector."""
        gripper_frame_id = self.robot.model.getFrameId("END_EFFECTOR")
        self.x = self.robot.data.oMf[gripper_frame_id].translation

    def update_M(self) -> np.ndarray:
        """Get the inertia matrix of the robot."""
        self.M = pinocchio.crba(self.robot.model, self.robot.data, self.q)

    def update_C(self) -> np.ndarray:
        """Get the coriolis/centrifugal matrix of the robot."""
        self.C = pinocchio.computeCoriolisMatrix(
            self.robot.model, self.robot.data, self.q, self.dq
        )

    def update_g(self) -> np.ndarray:
        """Get the gravity vector of the robot."""
        self.g = self.robot.gravity(self.q)

    def update_J(self) -> np.ndarray:
        """Get the Jacobian of the robot."""
        frame_id = self.robot.model.getFrameId("END_EFFECTOR")
        self.J = pinocchio.getFrameJacobian(
            self.robot.model,
            self.robot.data,
            frame_id,
            pinocchio.LOCAL_WORLD_ALIGNED,
        )[:3]

    def update_dJ(self) -> np.ndarray:
        """Get the derivative of the Jacobian of the robot."""
        frame_id = self.robot.model.getFrameId("END_EFFECTOR")
        self.dJ = pinocchio.getFrameJacobianTimeVariation(
            self.robot.model,
            self.robot.data,
            frame_id,
            pinocchio.LOCAL_WORLD_ALIGNED,
        )[:3]

    def update_target(self) -> np.ndarray:
        """Get the position of the target mocap body."""
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "target") - 1
        self.target = self.data.mocap_pos[body_id]

    def update_marker(
        self, marker: Literal["end_effector", "target"], pos: np.ndarray
    ) -> None:
        """Update the given marker."""
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, marker) - 1
        self.data.mocap_pos[body_id] = pos
