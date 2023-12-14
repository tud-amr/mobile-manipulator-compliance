import importlib.resources as pkg_resources
import os
import numpy as np
import casadi
from compliant_control.control.controller import Controller
import compliant_control.control.symbolics as symbolics
from dataclasses import dataclass


JOINTS = 6
WHEELS = 4
DIM = 3


@dataclass
class JointData:
    """Dataclass containing the position, velocity and current/torque of joints."""

    n: int

    def __post_init__(self) -> None:
        """Initialize vectors."""
        self.q = np.zeros(self.n)
        self.dq = np.zeros(self.n)
        self.c = np.zeros(self.n)


class State:
    """Contains the state of the robot."""

    def __init__(self, simulate: bool) -> None:
        self.load_symbolics()
        self.kinova_feedback = JointData(JOINTS)
        self.dingo_feedback = JointData(WHEELS)
        self.kinova_command = JointData(JOINTS)
        self.dingo_command = JointData(WHEELS)

        self.target = self.x

        self.controller = Controller(self)
        self.controller.reset()

        if simulate:
            self.ratios = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
            self.frictions = np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2])
        else:
            self.ratios = np.array([1.03, 0.31, 1.03, 1.90, 2.09, 1.99])
            self.frictions = np.array([0.54, 1.75, 0.69, 0.31, 0.40, 0.59])

    @property
    def g(self) -> np.ndarray:
        """Gravity vector."""
        return np.reshape(self.casadi_g(self.kinova_feedback.q), JOINTS)

    @property
    def x(self) -> np.ndarray:
        """Position of the end-effector."""
        return np.reshape(self.casadi_x(self.kinova_feedback.q), DIM)

    @property
    def dx(self) -> np.ndarray:
        """Velocity of the end-effector."""
        return np.reshape(
            self.casadi_dx(self.kinova_feedback.q, self.kinova_feedback.dq), DIM
        )

    @property
    def J(self) -> np.ndarray:
        """Jacobian."""
        return np.reshape(self.casadi_J(self.kinova_feedback.q), (DIM, JOINTS))

    @property
    def JT(self) -> np.ndarray:
        """Transposed Jacobian."""
        return np.reshape(self.casadi_JT(self.kinova_feedback.q), (JOINTS, DIM))

    @property
    def lam(self) -> np.ndarray:
        """Lambda."""
        return np.reshape(self.casadi_lam(self.kinova_feedback.q), (DIM, DIM))

    @property
    def mu(self) -> np.ndarray:
        """Mu."""
        return np.reshape(
            self.casadi_mu(self.kinova_feedback.q, self.kinova_feedback.dq), (DIM, DIM)
        )

    @property
    def N(self) -> np.ndarray:
        """Returns the null_space matrix."""
        return np.reshape(self.casadi_N(self.kinova_feedback.q), (JOINTS, JOINTS))

    @property
    def Nv(self) -> np.ndarray:
        """Returns the null_space matrix for velocity control."""
        return np.reshape(self.casadi_Nv(self.kinova_feedback.q), (JOINTS, JOINTS))

    def dq_inv(self, dx: np.ndarray) -> np.ndarray:
        """Joint velocities calculated using inverse kinematics."""
        return np.reshape(self.casadi_dq(self.kinova_feedback.q, dx), (JOINTS))

    def T(self, force: np.ndarray) -> np.ndarray:
        """Joint torques."""
        return np.reshape(self.casadi_T(self.kinova_feedback.q, force), (JOINTS))

    def load_symbolics(self) -> None:
        """Load the symbolics."""
        input_dir = str(pkg_resources.files(symbolics))
        current_dir = os.getcwd()
        os.chdir(input_dir)
        for file_name in os.listdir(input_dir):
            if file_name.endswith(".so"):
                name = file_name.replace(".so", "")
                f = casadi.external(name, file_name)
                setattr(self, f"casadi_{name}", f)
        os.chdir(current_dir)
