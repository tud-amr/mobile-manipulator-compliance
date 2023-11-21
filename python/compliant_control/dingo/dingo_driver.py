from compliant_control.dingo.lib.dingo_driver import DriverManager
from threading import Thread

from compliant_control.control.state import State
from compliant_control.utilities.rate_counter import RateCounter

GAIN = 0.4


class DingoDriver:
    """A class that uses the driver_manager to communicate with the robot."""

    def __init__(self, state: State) -> None:
        self.state = state
        self.log = lambda msg: print(msg)

        self.rate_counter = RateCounter(100)

        self.driver_manager = DriverManager("vcan0")
        self.driver_manager.connect_gateway()
        self.driver_manager.start_canread_loop()
        self.driver_manager.start_update_loop()

        thread = Thread(target=self.update_loop)
        thread.start()

    def update_loop(self) -> None:
        """Update the feedback and the command."""
        while True:
            self.state.dingo_feedback.q = list(self.driver_manager.position)
            self.state.dingo_feedback.c = list(self.driver_manager.torque)
            self.driver_manager.set_command(GAIN * self.state.dingo_command.c)
            self.rate_counter.count()
            self.rate_counter.sleep()
