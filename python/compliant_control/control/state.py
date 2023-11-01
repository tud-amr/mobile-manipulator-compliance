import importlib.resources as pkg_resources
import numpy as np
import casadi
import pinocchio
import pinocchio.casadi as cpin
import compliant_control.mujoco.models as models

JOINTS = 6


class State:
    """Contains the state of the robot."""

    def __init__(self) -> None:
        self.load_robot()
        self.define_matrices()
        self.q = np.zeros(JOINTS)
        self.dq = np.zeros(JOINTS)
        self.target = self.x

        self.simulation = True

    @property
    def ratios(self) -> np.ndarray:
        """Return the current torque ratios."""
        return (
            np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
            if self.simulation
            else np.array([1, 0.316, 1.02, 2.58, 2.02, 1])
        )

    @property
    def static_frictions(self) -> list:
        """Return the static frictions."""
        return (
            [0.2, 0.2, 0.2, 0.2, 0.2, 0.2]
            if self.simulation
            else [0.55, 1.1, 0.55, 0.11, 0.13, 0.33]
        )

    @property
    def dynamic_frictions(self) -> list:
        """Return the dynamic frictions."""
        return (
            [0.2, 0.2, 0.2, 0.2, 0.2, 0.2]
            if self.simulation
            else [0.622, 0.875, 0.747, 0.551, 0.694, 0.565]
        )

    @property
    def g(self) -> np.ndarray:
        """Gravity vector."""
        return self.robot.gravity(self.q)

    @property
    def x(self) -> np.ndarray:
        """Location of the end-effector."""
        self.robot.framesForwardKinematics(self.q)
        gripper_frame_id = self.robot.model.getFrameId("END_EFFECTOR")
        return self.robot.data.oMf[gripper_frame_id].translation

    @property
    def J(self) -> np.ndarray:
        """Jacobian."""
        return self.casadi_jacobian(self.q)

    @property
    def JT(self) -> np.ndarray:
        """Transposed Jacobian."""
        return self.casadi_jacobian_transposed(self.q)

    @property
    def lam(self) -> np.ndarray:
        """Lambda."""
        return self.casadi_lambda(self.q)

    @property
    def mu(self) -> np.ndarray:
        """Mu."""
        return self.casadi_mu(self.q, self.dq)

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
        self.robot.data = self.robot.model.createData()

    def define_matrices(self) -> None:
        """Create the symbolic matrices using Casadi."""
        model = cpin.Model(self.robot.model)
        data = model.createData()

        q = casadi.SX.sym("q", model.nq, 1)
        dq = casadi.SX.sym("dq", model.nq, 1)

        M = cpin.crba(model, data, q)
        Minv = casadi.solve(M, casadi.SX.eye(M.size1()))

        C = cpin.computeCoriolisMatrix(model, data, q, dq)

        frame_id = self.robot.model.getFrameId("END_EFFECTOR")
        J = cpin.getFrameJacobian(
            model,
            data,
            frame_id,
            pinocchio.LOCAL_WORLD_ALIGNED,
        )[:3, :]

        dJ = cpin.getFrameJacobianTimeVariation(
            model,
            data,
            frame_id,
            pinocchio.LOCAL_WORLD_ALIGNED,
        )[:3, :]

        Jinv = Minv @ J.T @ casadi.solve((J @ Minv @ J.T), casadi.SX.eye(3))

        lam = Jinv.T @ M @ Jinv
        mu = Jinv.T @ (C - M @ Jinv @ dJ) @ Jinv

        self.casadi_jacobian = casadi.Function("J", [q], [J])
        self.casadi_jacobian_transposed = casadi.Function("JT", [q], [J.T])
        self.casadi_lambda = casadi.Function("lam", [q], [lam])
        self.casadi_mu = casadi.Function("mu", [q, dq], [mu])
