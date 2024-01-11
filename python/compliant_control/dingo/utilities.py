import numpy as np


def direction_to_wheel_torques(direction: list) -> np.ndarray:
    """Calculate the required wheel torques to move in the given direction."""
    m = np.linalg.norm(direction)
    angle = np.arctan2(*direction)

    torque_A = calculate_torque(angle)
    torque_B = calculate_torque(-angle)
    return np.array([torque_A, torque_B, torque_B, torque_A]) * m


def rotation_to_wheel_torques(rotation: float) -> np.ndarray:
    """Calculate the required wheel torques to move in the given direction."""
    return np.array([1, -1, 1, -1]) * np.sign(rotation)


def calculate_torque(angle: float) -> float:
    """Calculate the required wheel torque to match the given moving angle."""
    if angle < -np.pi / 2:
        return -1
    if angle < 0:
        return 1 + angle * (4 / np.pi)
    if angle < np.pi / 2:
        return 1
    return 3 - angle * (4 / np.pi)
