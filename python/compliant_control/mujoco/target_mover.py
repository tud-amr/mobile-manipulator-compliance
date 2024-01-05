import numpy as np
from typing import Callable
from threading import Thread
import time

STEP_ANGLE = np.deg2rad(0.2)
ANGLE_MAX = np.deg2rad(90)
PAUSE = 2


class TargetMover:
    """A class that automatically moves the target."""

    def __init__(
        self,
        get_target: Callable[[np.ndarray], None],
        update_target: Callable[[np.ndarray], None],
    ) -> None:
        self.get_target = get_target
        self.update_target = update_target
        self.active = False
        self.rot_direction = 1
        self.z_direction = 1

    def move_target_loop(self) -> None:
        """A loop that moves the target."""
        x, y, z = self.get_target()
        at_zero = False
        while self.active:
            r = np.linalg.norm(np.sqrt(x**2 + y**2))
            angle = np.arctan2(y, x)
            if abs(angle) > ANGLE_MAX and self.rot_direction == np.sign(angle):
                time.sleep(PAUSE)
                self.rot_direction *= -1
                at_zero = False
            if abs(angle) < np.deg2rad(0.2) and not at_zero:
                time.sleep(PAUSE)
                at_zero = True
            new_angle = angle + self.rot_direction * STEP_ANGLE
            x = np.cos(new_angle) * r
            y = np.sin(new_angle) * r
            self.update_target([x, y, z])
            time.sleep(0.01)

    def toggle(self) -> None:
        """Toggle the active state."""
        self.active = not self.active
        if self.active:
            thread = Thread(target=self.move_target_loop)
            thread.start()
