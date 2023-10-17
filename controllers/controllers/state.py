import importlib.resources as pkg_resources
import numpy as np
import pinocchio
import kinova.models as models


class State:
    """Contains the state of the robot."""

    def __init__(self, simulation: bool, actuator_count: int) -> None:
        self.load_robot()

        self.target: np.ndarray
        self.x: np.ndarray
        self.M: np.ndarray
        self.C: np.ndarray
        self.g: np.ndarray
        self.J: np.ndarray
        self.dJ: np.ndarray

        self.active = [True] * actuator_count
        self.q = np.empty(actuator_count)
        self.dq = np.empty(actuator_count)
        self.current_torque_ratios = (
            {0: 1, 1: 1, 2: 1, 3: 1, 4: 1, 5: 1}
            if simulation
            else {0: 1, 1: 0.316, 2: 1.02, 3: 2.58, 4: 2.02, 5: 1}
        )
        self.dynamic_frictions = (
            {0: 0.2, 1: 0.2, 2: 0.2, 3: 0.2, 4: 0.2, 5: 0.2}
            if simulation
            else {0: 0.55, 1: 1.1, 2: 0.55, 3: 0.11, 4: 0.13, 5: 0.33}
        )
        self.static_frictions = (
            {0: 0.2, 1: 0.2, 2: 0.2, 3: 0.2, 4: 0.2, 5: 0.2}
            if simulation
            else {0: 0.622, 1: 0.875, 2: 0.747, 3: 0.551, 4: 0.694, 5: 0.565}
        )

    def update(self) -> None:
        """Update the state of the robot."""
        self.robot.forwardKinematics(self.q, self.dq)
        self.robot.computeJointJacobians(self.q)
        self.robot.framesForwardKinematics(self.q)

        self.update_x()
        self.update_M()
        self.update_C()
        self.update_g()
        self.update_J()
        self.update_dJ()

    def toggle_joint(self, joint: int) -> None:
        """Toggle active state of joint."""
        self.active[joint] = not self.active[joint]

    def get_joint_state(self, joint: int) -> bool:
        """Return whether the given joint is active."""
        return self.active[joint]

    def get_ratio(self, joint: int) -> float:
        """Get the current/torque ratio between the model and the real robot for the given joint."""
        return self.current_torque_ratios[joint]

    def get_dynamic_friction(self, joint: int) -> float:
        """Get the dynamic friction for the given joint."""
        return self.dynamic_frictions[joint]

    def get_static_friction(self, joint: int) -> float:
        """Get the static friction for the given joint."""
        return self.static_frictions[joint]

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
