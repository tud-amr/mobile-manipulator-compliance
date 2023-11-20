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

    def __init__(self) -> None:
        self.load_symbolics()
        self.kinova_feedback = JointData(JOINTS)
        self.dingo_feedback = JointData(WHEELS)
        self.kinova_command = JointData(JOINTS)
        self.dingo_command = JointData(WHEELS)

        self.target = self.x

        self.controller = Controller(self)
        self.controller.reset()

        self.simulation = True

    @property
    def ratios(self) -> np.ndarray:
        """Return the current torque ratios."""
        return (
            np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
            if self.simulation
            else np.array([1, 0.31, 1.01, 1.75, 1.75, 1])
        )

    @property
    def frictions(self) -> list:
        """Return the frictions."""
        return (
            [0.2, 0.2, 0.2, 0.2, 0.2, 0.2]
            if self.simulation
            else [0.61, 1.71, 0.72, 0.29, 0.41, 0.56]
        )

    @property
    def g(self) -> np.ndarray:
        """Gravity vector."""
        return np.reshape(self.casadi_g(self.kinova_feedback.q), JOINTS)

    @property
    def x(self) -> np.ndarray:
        """Location of the end-effector."""
        return np.reshape(self.casadi_x(self.kinova_feedback.q), DIM)

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
