from compliant_control.dingo.lib.dingo_driver import DriverManager
from dataclasses import dataclass
from typing import Literal
from threading import Thread

from compliant_control.control.state import State

GAIN = 0.2


@dataclass
class Wheel:
    """Contains the information of a wheel."""

    FR: Literal["front", "rear"]
    LR: Literal["left", "right"]
    canbus_id: int
    command: float = 0

    def __post_init__(self) -> None:
        """Define name."""
        self.name = f"{self.FR}_{self.LR}"


class DingoDriver:
    """A class that uses the driver_manager to communicate with the robot."""

    def __init__(self, state: State) -> None:
        self.state = State()
        self.driver_manager = DriverManager("vcan0")
        self.driver_manager.connect_gateway()

        canread_thread = Thread(target=self.canread_loop)
        canread_thread.start()

        self.add_actuators()

        update_thread = Thread(target=self.update_loop)
        update_thread.start()

    def add_actuators(self) -> None:
        """Add the actuators."""
        self.wheels = [
            Wheel("front", "left", 2),
            Wheel("front", "right", 3),
            Wheel("rear", "left", 4),
            Wheel("rear", "right", 5),
        ]
        for wheel in self.wheels:
            self.driver_manager.add_actuator(
                wheel.canbus_id, wheel.name, wheel.LR == "right"
            )
            self.driver_manager.set_mode(wheel.name, "Vol")
        self.driver_manager.initialize_encoders()

    def command(self) -> None:
        """Send a command."""
        command = [torque * GAIN for torque in self.state.dingo_command.c]
        for n, wheel in enumerate(self.wheels):
            self.driver_manager.command(wheel.name, "Vol", command[n])

    def update_loop(self) -> None:
        """Update the feedback and send command."""
        while self.active:
            states = self.driver_manager.get_states()
            for n, state in enumerate(states):
                self.state.dingo_feedback.q[n] = state.position
                self.state.dingo_feedback.dq[n] = state.speed
                self.state.dingo_feedback.c[n] = state.current * state.voltage
            self.command()

    def canread_loop(self) -> None:
        """A loop that reads the canbus."""
        self.active = True
        while self.active:
            self.driver_manager.canread()
