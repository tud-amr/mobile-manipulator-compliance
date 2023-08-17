import numpy as np
import pandas as pd
import importlib.resources as pkg_resources
import calibration_data
from scipy.signal import savgol_filter
from user_interface.logger import Logger

from compliant_controller.controller import Controller
from kinova.kortex_client import KortexClient
from kinova.specifications import Position
import time


ZERO_ERROR = 10**-2


class Calibration:
    """Class used for all calibration."""

    def __init__(self, client: KortexClient) -> None:
        self.client = client
        self.friction = CalibrateFriction(client)
        self.gravity = CalibrateGravity(client)

    def calibrate_friction(self) -> None:
        """Start the friction calibration."""
        self.client.calibrating = True
        self.client._high_level_move(Position("0,3,5", [0, 0, 0, 0, 0, 0]))
        self.friction.calibrate(0)
        self.friction.calibrate(3)
        self.friction.calibrate(5)
        self.client._high_level_move(Position("4", [0, 0, 90, 0, 0, 0]))
        self.friction.calibrate(4)
        self.client.calibrating = False

    def calibrate_gravity(self) -> None:
        """Start the gravity calibration."""
        self.client.calibrating = True
        self.client._high_level_move(Position("1", [0, 90, 90, 0, 0, 0]))
        self.gravity.calibrate(1)
        self.client._high_level_move(Position("2", [0, 0, 90, 0, 0, 0]))
        self.gravity.calibrate(2)
        self.client._high_level_move(Position("3", [0, 0, 90, 0, 90, 0]))
        self.gravity.calibrate(3)
        self.client._high_level_move(Position("4", [0, 0, 0, 0, 90, 0]))
        self.gravity.calibrate(4)
        self.gravity.export_data()
        self.client.calibrating = False


class CalibrateFriction(Controller):
    """Calibrate the static friction of the robot."""

    def __init__(self, client: KortexClient) -> None:
        super().__init__(client)
        self.pause_till = time.time()
        self.zero_velocity_time = time.time()
        self.velocity_time = time.time()
        self.calibration_step = 0.0001

    def non_zero_velocity(self) -> None:
        """Return true if the joint has non_zero velocity for at least one second."""
        if abs(self.state.dq[self.joint]) < ZERO_ERROR:
            self.velocity_time = time.time()
        else:
            self.pause_till = time.time() + 0.1
        return time.time() > self.velocity_time + 0.1

    def command(self) -> None:
        """Move to calibration position or calibrate joint."""
        super().command()
        self.commands[0] = self.torque
        if self.calibrating:
            if self.non_zero_velocity():
                Logger.log(f"Moved at {self.torque} torque.")
                self.client.disconnect_LLC()
                self.calibrating = False
            else:
                if time.time() > self.pause_till:
                    self.torque += self.calibration_step

    def calibrate(self, joint: int) -> None:
        """Reset before connecting to LLC."""
        self.joint = joint
        self.state.active = [False] * self.client.actuator_count
        self.state.active[self.joint] = True
        self.joints = [n for n, active in enumerate(self.state.active) if active]
        self.torque = 0
        self.client._start_LLC()
        self.client._connect_LLC(self, "current")
        self.calibrating = True
        while self.calibrating:
            time.sleep(1)
        Logger.log(f"Friction calibration of joint {joint} done.")
        self.client._stop_LLC()
        return


class CalibrateGravity(Controller):
    """Calibrate the gravity of the robot."""

    def __init__(self, client: KortexClient) -> None:
        super().__init__(client)
        self.client = client
        self.record_steps = client.frequency * 3
        self.data = np.zeros((client.actuator_count, 2, self.record_steps))

    def calibrate(self, joint: int) -> None:
        """Reset before connecting to LLC."""
        self.joint = joint
        self.n = 0
        time.sleep(1)
        while self.n < self.record_steps:
            self.record_data()
            time.sleep(1 / self.client.frequency)
        Logger.log(f"Gravity calibration of joint {joint} done.")
        return

    def finish_calibration(self) -> None:
        """Finish the calibration process."""
        Logger.log(f"Calibration of joint {self.joint} finished.")
        Logger.log("Calibration finished.")
        self.export_data()
        self.client.disconnect_LLC()

    def record_data(self) -> None:
        """Record the current data."""
        self.data[self.joint][0][self.n] = self.state.g[self.joint]
        self.data[self.joint][1][self.n] = (
            self.client.get_torque(self.joint, False)
            if self.client.mock
            else self.client.get_current(self.joint, False)
        )
        self.n += 1

    def export_data(self) -> None:
        """Export the data."""
        directory = str(pkg_resources.files(calibration_data))
        data = {}
        for n, joint_data in enumerate(self.data):
            if all(joint_data[0]):
                data[f"Model joint {n}"] = joint_data[0]
                data[f"Actual joint {n}"] = joint_data[1]
                data[f"Filtered joint {n}"] = savgol_filter(joint_data[1], 51, 2)
                data[f"Ratio joint {n}"] = data[f"Filtered joint {n}"] / joint_data[0]
                mean = np.mean(data[f"Ratio joint {n}"])
                data[f"RatioMean joint {n}"] = np.full_like(joint_data[0], mean)
                self.state.current_torque_ratios[n] = mean
            else:
                data[f"Model joint {n}"] = 0
                data[f"Actual joint {n}"] = 0
                data[f"Filtered joint {n}"] = 0
                data[f"Ratio joint {n}"] = 0
                data[f"RatioMean joint {n}"] = 0
                self.state.current_torque_ratios[n] = 0
        df = pd.DataFrame(data)
        df.to_csv(directory + "/data.csv", index=False)
