from threading import Thread
from compliant_control.control.state import State
from compliant_control.mujoco.simulation import Simulation
from compliant_control.utilities.rate_counter import RateCounter

GAIN = 3


class DingoDriverSimulation:
    """A simulation of the Dingo driver."""

    def __init__(self, state: State, simulation: Simulation) -> None:
        self.state = state
        self.simulation = simulation

        self.rate_counter = RateCounter(100)
        self.start_update_loop()

    def start_update_loop(self) -> None:
        """Start the update loop."""
        self.active = True
        update_thread = Thread(target=self.update_loop)
        update_thread.start()

    def update_loop(self) -> None:
        """Update loop."""
        while self.active:
            self.update()
            self.command()
            self.rate_counter.count()
            self.rate_counter.sleep()

    def command(self) -> None:
        """Send a command."""
        command = [torque * GAIN for torque in self.state.dingo_command.c]
        self.simulation.set_ctrl_value("Dingo", "torque", command)

    def update(self) -> None:
        """Update the state."""
        self.state.dingo_feedback.q = self.simulation.get_sensor_feedback(
            "Dingo", "position"
        )
        self.state.dingo_feedback.dq = self.simulation.get_sensor_feedback(
            "Dingo", "velocity"
        )
        self.state.dingo_feedback.c = self.simulation.get_sensor_feedback(
            "Dingo", "torque"
        )
