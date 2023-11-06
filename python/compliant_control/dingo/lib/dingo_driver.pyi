from typing import Literal
from dataclasses import dataclass


@dataclass
class State:
    """Contains the state elements of an actuator."""

    name: str
    position: float
    speed: float
    voltage: float
    current: float


class DriverManager:
    """Manages the canbus connections."""

    def __init__(self, canbus_name: str) -> None:
        """Define the canbus name."""

    def connect_gateway(self) -> None:
        """Connect the gateway."""

    def add_actuator(self, can_id: int, name: str, flip: bool) -> None:
        """Add an actuator to the driver manager."""

    def set_mode(self, name: str, mode: Literal["Vol", "Cur", "Spd"]) -> None:
        """Set the mode of an actuator."""

    def initialize_encoders(self) -> None:
        """Initialize the encoders for all actuators."""

    def command(
        self, name: str, mode: Literal["Vol", "Cur", "Spd"], value: float
    ) -> None:
        """Send a command. Voltage as value between -1 and 1."""

    def canread(self) -> None:
        """Reads the canbus for all actuators."""

    def get_states(self) -> list[State]:
        """Get the states of all actuators."""
