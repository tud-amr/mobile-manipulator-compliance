import numpy as np
import pandas as pd
import importlib.resources as pkg_resources
import calibration_data
from scipy.signal import savgol_filter
from user_interface.logger import Logger

from compliant_controller.controller import Controller
from kinova.kortex_client import KortexClient
import time


JOINT_POSES = [
    [0, 0, 0, 0, 0, 0],
    [0, np.pi / 2, np.pi / 2, 0, 0, 0],
    [0, 0, np.pi / 2, 0, 0, 0],
    [0, 0, np.pi / 2, 0, np.pi / 2, 0],
    [0, 0, 0, 0, np.pi / 2, 0],
    [0, 0, 0, 0, 0, 0],
]

CALIBRATION_TIME = 2  # s
MOVING_SPEED = 20  # deg/s
START = 500
END = 1500
REACHED_ERROR = 10**-5
NEARBY_GOAL_DIVIDER = 100
SKIP_JOINTS = [0, 5]


class Calibration(Controller):
    """Class used to calibrate the robot."""

    def __init__(self, client: KortexClient) -> None:
        super().__init__(client)
        self.mode = "position"
        self.define_step_size()

    def define_step_size(self) -> None:
        """Define the step size."""
        step_deg = MOVING_SPEED / self.client.frequency
        step_rad = np.deg2rad(step_deg)
        self.step_size = step_rad

    def connect_to_LLC(self) -> None:
        """Reset before connecting to LLC."""
        self.commands = self.state.q.copy()
        self.state.active = [True] * self.client.actuator_count
        self.data = np.zeros((len(JOINT_POSES), 2, END - START))
        self.joint = -1
        self.record_till = time.time()
        self.go_to_next_joint()
        super().connect_to_LLC()

    def command(self) -> None:
        """Move to calibration position or calibrate joint."""
        super().command()
        if time.time() <= self.record_till:
            self.record_data()
            self.n += 1
            return
        if self.ready:
            self.calibrate_joint()
        else:
            self.ready = self.move_to_pose(JOINT_POSES[self.joint])

    def calibrate_joint(self) -> None:
        """Calibrate the joint."""
        if self.joint_finished:
            if self.joint < len(JOINT_POSES) - 1:
                self.go_to_next_joint()
            else:
                self.finish_calibration()
        else:
            Logger.log(f"Calibrating joint {self.joint}...")
            if self.joint not in SKIP_JOINTS:
                self.record_till = time.time() + CALIBRATION_TIME
            self.joint_finished = True

    def finish_calibration(self) -> None:
        """Finish the calibration process."""
        Logger.log(f"Calibration of joint {self.joint} finished.")
        Logger.log("Calibration finished.")
        self.export_data()
        self.client.disconnect_LLC()

    def go_to_next_joint(self) -> None:
        """Go to the next joint to calibrate."""
        self.n = 0
        self.ready = False
        self.joint_finished = False
        self.reached = [False] * self.client.actuator_count
        Logger.log(f"Calibration of joint {self.joint} finished.")
        self.joint += 1

    def move_to_pose(self, pose: list) -> bool:
        """Move to pose."""
        for n in range(self.client.actuator_count):
            if self.reached[n]:
                continue
            error = abs(pose[n] - self.state.q[n])
            if error < REACHED_ERROR:
                self.reached[n] = True
            else:
                self.create_command(n, pose, error)
        return all(self.reached)

    def create_command(self, joint: int, pose: list, error: float) -> float:
        """Create the command."""
        step = min(error / NEARBY_GOAL_DIVIDER, self.step_size)
        if self.state.q[joint] > pose[joint]:
            self.commands[joint] -= step
        elif self.state.q[joint] < pose[joint]:
            self.commands[joint] += step

    def record_data(self) -> None:
        """Record the current data."""
        if START <= self.n < END:
            n = self.n - START
            self.data[self.joint][0][n] = self.state.g[self.joint]
            self.data[self.joint][1][n] = (
                self.client.get_torque(self.joint, False)
                if self.client.mock
                else self.client.get_current(self.joint, False)
            )

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
