import importlib.resources as pkg_resources
import os
import numpy as np
import casadi
from compliant_control.control.controller import Controller
import compliant_control.control.symbolics as symbolics
from dataclasses import dataclass


JOINTS = 6
DIM = 3


@dataclass
class JointData:
    """Dataclass containing the position, velocity and current/torque of joints."""

    q = np.zeros(JOINTS)
    dq = np.zeros(JOINTS)
    c = np.zeros(JOINTS)


class State:
    """Contains the state of the robot."""

    def __init__(self) -> None:
        self.load_symbolics()
        self.feedback = JointData()
        self.command = JointData()
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
        return np.reshape(self.casadi_g(self.feedback.q), JOINTS)

    @property
    def x(self) -> np.ndarray:
        """Location of the end-effector."""
        return np.reshape(self.casadi_x(self.feedback.q), DIM)

    @property
    def J(self) -> np.ndarray:
        """Jacobian."""
        return np.reshape(self.casadi_J(self.feedback.q), (DIM, JOINTS))

    @property
    def JT(self) -> np.ndarray:
        """Transposed Jacobian."""
        return np.reshape(self.casadi_JT(self.feedback.q), (JOINTS, DIM))

    @property
    def lam(self) -> np.ndarray:
        """Lambda."""
        return np.reshape(self.casadi_lam(self.feedback.q), (DIM, DIM))

    @property
    def mu(self) -> np.ndarray:
        """Mu."""
        return np.reshape(self.casadi_mu(self.feedback.q, self.feedback.dq), (DIM, DIM))

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
