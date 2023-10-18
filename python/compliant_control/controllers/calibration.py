import numpy as np
import pandas as pd
import importlib.resources as pkg_resources

# import calibration_data
from scipy.signal import savgol_filter
from scipy.optimize import minimize

from compliant_control.kinova.kortex_client import KortexClient
from compliant_control.kinova.specifications import Position
from compliant_control.controllers.state import State
from compliant_control.controllers.controllers import CompensateGravity
import time
from threading import Thread


class Calibrations:
    """Contains all the calibrations."""

    def __init__(self, state: State, client: KortexClient) -> None:
        self.low_level = LowLevelCalibration(state, client)
        self.high_level = HighLevelCalibration(state, client)


class LowLevelCalibration(CompensateGravity):
    """Class used for calibration of static friction."""

    def __init__(self, state: State, client: KortexClient) -> None:
        super().__init__(client)
        self.state = state
        self.client = client
        self.tolerance = 0.01

    def calibrate_all_joints(self) -> None:
        """Calibrate all joints."""
        self.client.calibrating = True
        self.calibrate(0, [0, 0, 0, 0, 0, 0])
        self.calibrate(1, [0, 0, 0, 90, 0, 0])
        self.calibrate(2, [0, 0, 0, 90, 0, 0])
        self.calibrate(3, [0, 0, 0, 0, 0, 0])
        self.calibrate(4, [0, 0, 90, 0, 0, 0])
        self.calibrate(5, [0, 0, 0, 0, 0, 0])
        self.client.calibrating = False

    def calibrate(self, joint: int, start_pos: list) -> None:
        """Calibrate the given joint."""
        self.joint = joint
        self.state.active = [n == joint for n in range(self.client.actuator_count)]
        position = start_pos
        calibrations = 1 if self.client.mock else 4
        angles = np.linspace(-90, 90, calibrations)

        measured_frictions = []
        for angle in angles:
            self.torque = 0
            self.moved = False
            self.started = False
            self.finished = False
            position[joint] = angle
            self.client._high_level_move(Position("", position))
            self.client._start_LLC()
            self.client._connect_LLC(self)
            time.sleep(1)
            self.started = True
            while not self.finished:
                time.sleep(0.1)
            measured_frictions.append(self.torque)

        average_friction = np.mean(measured_frictions)
        measured_frictions = np.around(measured_frictions, 3)
        print(f"Joint {joint}: {measured_frictions}, {average_friction}")

    def command(self) -> None:
        """Increase torque until it moves."""
        super().command()

        if not self.started:
            return
        if self.moved:
            if abs(self.state.dq[self.joint]) < self.tolerance:
                self.client._disconnect_LLC()
                self.client._stop_LLC()
                self.finished = True
            return
        if abs(self.state.dq[self.joint]) < self.tolerance:
            self.torque += 0.0001
        else:
            self.moved = True

        self.commands[0] += self.torque


class HighLevelCalibration:
    """Class used for calibration of torque/current ratio and dynamic friction."""

    def __init__(self, state: State, client: KortexClient) -> None:
        self.state = state
        self.client = client

    def calibrate_all_joints(self) -> None:
        """Calibrate all joints."""
        self.client.calibrating = True
        for joint in range(self.client.actuator_count):
            self.calibrate(joint)
        self.client.calibrating = False

    def calibrate(self, joint: int) -> None:
        """Calibrate the given joint."""
        self.client.calibrating = True
        self.joint = joint
        start = [0 if joint != n else -90 for n in range(self.client.actuator_count)]
        end = [0 if joint != n else 90 for n in range(self.client.actuator_count)]

        # Joint 3 requires a modification of the pose:
        if joint == 3:
            start[2] = 90
            end[2] = 90

        self.client._high_level_move(Position("", start))
        self.start_calibration()
        self.client._high_level_move(Position("", end))
        self.stop_calibration()
        self.client.calibrating = False

    def start_calibration(self) -> None:
        """Start the calibration."""
        self.velocity = []
        self.gravity = []
        self.torque = []
        thread = Thread(target=self.record_data)
        thread.start()

    def stop_calibration(self) -> None:
        """Stop the calibration."""
        self.calibrating = False
        # self.export_data()

    def record_data(self) -> None:
        """Record the data."""
        self.calibrating = True
        while self.calibrating:
            self.velocity.append(self.state.dq[self.joint])
            self.gravity.append((self.state.g[self.joint]))
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
        pre = "mock_" if self.client.mock else ""
        df.to_csv(directory + f"/{pre}calibration_{self.joint}.csv", index=False)

    def _f_scale_and_move(self, params: list) -> None:
        a, b = params
        return np.sqrt(np.sum((self.gravity - (a * self.torque + b)) ** 2))

    def _f_move(self, b: float) -> None:
        return np.sqrt(np.sum((self.gravity - (self.torque + b)) ** 2))
