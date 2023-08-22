import numpy as np
import pandas as pd
import importlib.resources as pkg_resources
import calibration_data
from scipy.signal import savgol_filter
from scipy.optimize import minimize

from kinova.kortex_client import KortexClient
from kinova.specifications import Position
import time
from threading import Thread


class Calibration:
    """Class used for all calibration."""

    def __init__(self, client: KortexClient) -> None:
        self.client = client

    def calibrate_all_joints(self) -> None:
        """Calibrate all joints."""
        for joint in range(self.client.actuator_count):
            self.calibrate(joint)

    def calibrate(self, joint: int) -> None:
        """Calibrate the given joint."""
        self.client.calibrating = True
        start = [0 if joint != n else -90 for n in range(self.client.actuator_count)]
        end = [0 if joint != n else 90 for n in range(self.client.actuator_count)]

        # Joint 3 requires a modification of the pose:
        if joint == 3:
            start[2] = 90
            end[2] = 90

        self.client._high_level_move(Position("", start))
        self.start_calibration(joint)
        self.client._high_level_move(Position("", end))
        self.stop_calibration()
        self.client.calibrating = False

    def start_calibration(self, joint: int) -> None:
        """Start the calibration."""
        self.joint = joint
        self.velocity = []
        self.gravity = []
        self.torque = []
        thread = Thread(target=self.record_data)
        thread.start()

    def stop_calibration(self) -> None:
        """Stop the calibration."""
        self.calibrating = False
        self.export_data()

    def record_data(self) -> None:
        """Record the data."""
        self.calibrating = True
        while self.calibrating:
            self.velocity.append(self.client.state.dq[self.joint])
            self.gravity.append((self.client.state.g[self.joint]))
            if self.client.mock:
                self.torque.append((self.client.get_torque(self.joint, False)))
            else:
                self.torque.append((self.client.get_current(self.joint, False)))
            time.sleep(1 / self.client.frequency)

    def export_data(self) -> None:
        """Export the data."""
        directory = str(pkg_resources.files(calibration_data))
        p25 = int(len(self.velocity) * 0.25)

        velocity = savgol_filter(self.velocity, 501, 2)[p25:-p25]
        self.gravity = np.array(self.gravity[p25:-p25])
        self.torque = savgol_filter(self.torque, 501, 2)[p25:-p25]

        gravity = sum(abs(self.gravity)) > 1

        if not gravity:
            result = minimize(self._f_move, 0)
            ratio = 1
            friction = result.x[0]
        else:
            result = minimize(self._f_scale_and_move, [1, 0])
            ratio = result.x[0]
            friction = result.x[1]

        print(f"Joint {self.joint}: {1/ratio, friction}")

        data = {
            "Velocity": velocity,
            "Gravity": self.gravity,
            "Torque": self.torque,
            "Scaled": ratio * self.torque,
            "Transformed": ratio * self.torque + friction,
        }
        df = pd.DataFrame(data)
        pre = "mock" if self.client.mock else "robot"
        df.to_csv(directory + f"/{pre}_calibration_{self.joint}.csv", index=False)

    def _f_scale_and_move(self, params: list) -> None:
        a, b = params
        return np.sqrt(np.sum((self.gravity - (a * self.torque + b)) ** 2))

    def _f_move(self, b: float) -> None:
        return np.sqrt(np.sum((self.gravity - (self.torque + b)) ** 2))
