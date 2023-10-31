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
    active: bool = False
    mode: str = "?"
    ratio: float = 0
    fric_d: float = 0
    fric_s: float = 0

    @property
    def name(self) -> str:
        """Return the name of the joint."""
        return f"joint{self.index}"

    def is_active(self) -> bool:
        """Return wether the joint is active."""
        return self.active

    def get_mode(self) -> str:
        """Return the mode of the joint."""
        return self.mode

    def get_ratio(self) -> str:
        """Return the ratio of the joint."""
        return str(self.ratio)

    def get_fric_d(self) -> str:
        """Return the dynamic friction of the joint."""
        return str(round(self.fric_d, 3))

    def get_fric_s(self) -> str:
        """Return the static friction of the joint."""
        return str(round(self.fric_s, 3))


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
class Rate:
    """Object to collect update rate."""

    n: int = 0
    last: int = 0

    def inc(self) -> str:
        """Increment n."""
        self.n += 1

    def get_rate(self) -> str:
        """Get the last rate."""
        return str(self.last)

    def update(self) -> None:
        """Update the rate."""
        self.last = self.n
        self.n = 0


@dataclass
class State:
    """Data of the robot state."""

    mode: str = "waiting"
    update_rate: int = 0
    servoing: str = "?"
    comp_grav: bool = False
    comp_fric: bool = False
    imp_joint: bool = False
    imp_cart: bool = False
    move_tar: bool = False

    def HLC(self) -> bool:
        """Returns whether the current mode is HLC."""
        return self.mode == "HLC"

    def LLC(self) -> bool:
        """Returns whether the current mode is LLC."""
        return self.mode == "LLC"

    def LLC_task(self) -> bool:
        """Returns whether a LLC task is active."""
        return self.mode == "LLC_task"

    def get_rate(self) -> str:
        """Get the update rate."""
        return str(self.update_rate)

    def get_servoing(self) -> str:
        """Get the servoing mode."""
        return self.servoing

    def get_comp_grav(self) -> bool:
        """Return whether gravity compensation is enabled."""
        return self.comp_grav

    def get_comp_fric(self) -> bool:
        """Return whether friction compensation is enabled."""
        return self.comp_fric

    def get_imp_joint(self) -> bool:
        """Return whether joint impedance is enabled."""
        return self.imp_joint

    def get_imp_cart(self) -> bool:
        """Return whether cartesian impedance is enabled."""
        return self.imp_cart
