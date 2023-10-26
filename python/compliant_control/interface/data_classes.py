import numpy as np
from dataclasses import dataclass


@dataclass
class Joint:
    """Data of a joint."""

    index: int

    # continuous feedback:
    pos: float = 0
    vel: float = 0
    eff: float = 0

    # state:
    active: bool = True
    mode: str = "?"
    ratio: float = 0
    fric_d: float = 0
    fric_s: float = 0

    @property
    def name(self) -> str:
        """Return the name of the joint."""
        return f"joint{self.index}"


@dataclass
class Wheel:
    """Data of a wheel."""

    index: int

    # continuous feedback:
    encoder_position: float = 0
    zero_position: float = 0
    vel: float = 0
    eff: float = 0

    @staticmethod
    def calculate_torques(direction: list) -> list:
        """Calculate the required wheel torques to move in the given direction."""
        m = np.linalg.norm(direction)
        angle = np.arctan2(*direction)

        orientations = ["l", "r", "r", "l"]
        torques = []
        for n in range(4):
            orientation = orientations[n]
            torque = Wheel.calculate_torque(angle, orientation) * m
            torques.append(torque)
        return torques

    @staticmethod
    def calculate_torque(angle: float, orientation: str) -> float:
        """Calculate the required wheel torque to match the given moving angle."""
        torques = [-1, -1, -1, 0, 1, 1, 1, 0, -1]
        angles = np.linspace(-np.pi, np.pi, 9)
        if orientation == "r":
            torques.reverse()
        return np.interp(angle, angles, torques)

    @property
    def name(self) -> str:
        """Return the name of the wheel."""
        return f"wheel{self.index}"

    @property
    def pos(self) -> float:
        """Return the current position."""
        return self.encoder_position - self.zero_position

    def reset(self) -> None:
        """Reset the position."""
        self.zero_position = self.encoder_position


@dataclass
class State:
    """Data of the robot state."""

    mode: str = "waiting"
    update_rate: int = 0
    servoing: str = "?"
    comp_fric: bool = False
    move_tar: bool = False

    @property
    def HLC(self) -> bool:
        """Returns whether the current mode is HLC."""
        return self.mode == "HLC"

    @property
    def LLC(self) -> bool:
        """Returns whether the current mode is LLC."""
        return self.mode == "LLC"

    @property
    def LLC_task(self) -> bool:
        """Returns whether a LLC task is active."""
        return self.mode == "LLC_task"
