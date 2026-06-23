import os
import numpy as np
import casadi
from compliant_control.control.controller import Controller
import compliant_control.control.symbolics as symbolics
from dataclasses import dataclass

JOINTS = 6
WHEELS = 4
DIM = 3

def rotMatrix_to_quaternion(rotation_matrix: np.ndarray, ordering="xyzw") -> np.ndarray:
    """
    Convert a 4x4 transformation matrix to a quaternion.

    Parameters:
        matrix (numpy.ndarray): The 4x4 transformation matrix.

    Returns:
        numpy.ndarray: The quaternion representation (w, x, y, z).
    """

    # Calculate the trace of the rotation matrix
    trace = np.trace(rotation_matrix)

    if trace > 0:
        # The quaternion calculation when the trace is positive
        s = np.sqrt(trace + 1.0) * 2
        w = 0.25 * s
        x = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
        y = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
        z = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
    elif (rotation_matrix[0, 0] > rotation_matrix[1, 1]) and (
        rotation_matrix[0, 0] > rotation_matrix[2, 2]
    ):
        # The quaternion calculation when the trace is largest along x-axis
        s = (
            np.sqrt(
                1.0
                + rotation_matrix[0, 0]
                - rotation_matrix[1, 1]
                - rotation_matrix[2, 2]
            )
            * 2
        )
        w = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
        x = 0.25 * s
        y = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
        z = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
    elif rotation_matrix[1, 1] > rotation_matrix[2, 2]:
        # The quaternion calculation when the trace is largest along y-axis
        s = (
            np.sqrt(
                1.0
                + rotation_matrix[1, 1]
                - rotation_matrix[0, 0]
                - rotation_matrix[2, 2]
            )
            * 2
        )
        w = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
        x = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
        y = 0.25 * s
        z = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
    else:
        # The quaternion calculation when the trace is largest along z-axis
        s = (
            np.sqrt(
                1.0
                + rotation_matrix[2, 2]
                - rotation_matrix[0, 0]
                - rotation_matrix[1, 1]
            )
            * 2
        )
        w = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
        x = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
        y = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
        z = 0.25 * s

    quaternion = np.array([1, 0, 0, 0])
    if ordering == "wxyz":
        quaternion = np.array([w, x, y, z])
    elif ordering == "xyzw":
        quaternion = np.array([x, y, z, w])
    else:
        raise InvalidQuaternionOrderError(
            f"Order {ordering} is not permitted, options are 'xyzw', and 'wxyz'"
        )

    return quaternion



@dataclass
class JointData:
    """Dataclass containing the position, velocity and current/torque of joints."""

    n: int

    def __post_init__(self) -> None:
        """Initialize vectors."""
        self.q = np.zeros(self.n)
        self.dq = np.zeros(self.n)
        self.c = np.zeros(self.n)
        self.fault = np.zeros(self.n)


class State:
    """Contains the state of the robot."""

    def __init__(self, simulate: bool) -> None:
        self.load_symbolics()
        self.kinova_feedback = JointData(JOINTS)
        self.dingo_feedback = JointData(WHEELS)
        self.kinova_command = JointData(JOINTS)
        self.dingo_command = JointData(WHEELS)

        self.target = self.x
        self.absolute_target = self.x
        self.target_q = np.array(self.kinova_feedback.q)
        self.pose_base = np.zeros(3)

        self.controller = Controller(self)
        self.controller.reset()

        if simulate:
            self.ratios = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
            self.frictions = np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2])
        else:
            self.ratios = np.array([1.0282, 0.3074, 1.0282, 1.9074, 2.0373, 1.9724])
            self.frictions = np.array([0.5318, 1.4776, 0.6695, 0.3013, 0.3732, 0.5923])

    def reset_target(self):
        self.target = self.x
        self.absolute_target = self.x
        self.target_q = self.kinova_feedback.q
        
    @property
    def g(self) -> np.ndarray:
        """Gravity vector."""
        return np.reshape(self.casadi_g(self.kinova_feedback.q), JOINTS)

    @property
    def x(self) -> np.ndarray:
        """Position of the end-effector."""
        return np.reshape(self.casadi_x(self.kinova_feedback.q), DIM)

    @property
    def quat(self) -> np.ndarray:
        """Rotation matrix of the end-effector."""
        rot_matrix = np.reshape(self.casadi_rot(self.kinova_feedback.q), (DIM, DIM))
        quaternion = rotMatrix_to_quaternion(rot_matrix)
        return quaternion

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
        input_dir = os.path.dirname(symbolics.__file__)
        current_dir = os.getcwd()
        os.chdir(input_dir)
        for file_name in os.listdir(input_dir):
            if file_name.endswith(".so"):
                name = file_name.replace(".so", "")
                f = casadi.external(name, file_name)
                setattr(self, f"casadi_{name}", f)
        os.chdir(current_dir)
