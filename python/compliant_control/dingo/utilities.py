import numpy as np


def direction_to_wheel_torques(direction: list) -> np.ndarray:
    """Calculate the required wheel torques to move in the given direction."""
    m = np.linalg.norm(direction)
    angle = np.arctan2(*direction)

    orientations = ["l", "r", "r", "l"]
    torques = []
    for orientation in orientations:
        torque = calculate_torque(angle, orientation) * m
        torques.append(torque)
    return np.array(torques)


def calculate_torque(angle: float, orientation: str) -> float:
    """Calculate the required wheel torque to match the given moving angle."""
    torques = [-1, -1, -1, 0, 1, 1, 1, 0, -1]
    angles = np.linspace(-np.pi, np.pi, 9)
    if orientation == "r":
        torques.reverse()
    return np.interp(angle, angles, torques)
