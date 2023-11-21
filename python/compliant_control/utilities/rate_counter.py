from threading import Thread
import time


class RateCounter:
    """Class used to count the rate of loops."""

    def __init__(self, desired_rate: int) -> None:
        self.desired_rate = desired_rate
        self.n = 0
        self.rate = 0
        self.sleep_time = 1 / self.desired_rate
        self.start_loop()

    def count(self) -> None:
        """Increase n."""
        self.n += 1

    def sleep(self) -> None:
        """Sleep to match desired rate."""
        time.sleep(self.sleep_time)

    def start_loop(self) -> None:
        """Start the loop."""
        self.active = True
        thread = Thread(target=self.loop)
        thread.start()

    def stop_loop(self) -> None:
        """Stop the loop."""
        self.active = False

    def loop(self) -> None:
        """Loop that updates the rate every second."""
        while self.active:
            if self.n != 0:
                self.rate = self.n
                self.n = 0
                self.sleep_time *= self.rate / self.desired_rate
            time.sleep(1)
