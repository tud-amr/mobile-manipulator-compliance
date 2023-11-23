import numpy as np
from typing import Callable
from threading import Thread
import time

STEP_ANGLE = np.deg2rad(0.2)
STEP_Z = 0.0005
ANGLE_MAX = np.deg2rad(130)
Z0 = 0.4
Z_MAX = 0.2


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
        while self.active:
            x, y, z = self.get_target()
            r = np.linalg.norm(np.sqrt(x**2 + y**2))
            angle = np.arctan2(y, x)
            if abs(angle) > ANGLE_MAX and self.rot_direction == np.sign(angle):
                self.rot_direction *= -1
            if abs(z - Z0) > Z_MAX and self.z_direction == np.sign(z - Z0):
                self.z_direction *= -1
            new_angle = angle + self.rot_direction * STEP_ANGLE
            x = np.cos(new_angle) * r
            y = np.sin(new_angle) * r
            z += self.z_direction * STEP_Z
            self.update_target([x, y, z])
            time.sleep(0.01)

    def toggle(self) -> None:
        """Toggle the active state."""
        self.active = not self.active
        if self.active:
            thread = Thread(target=self.move_target_loop)
            thread.start()
