import numpy as np
import pandas as pd
import importlib.resources as pkg_resources

from scipy.signal import savgol_filter
from scipy.optimize import minimize

from compliant_control.kinova.kortex_client import KortexClient
from compliant_control.kinova.specifications import Position
from compliant_control.control.state import State
from compliant_control.control import calibration_data

import time
from threading import Thread

JOINTS = 6


class Calibration:
    """Contains all the calibrations."""

    def __init__(self, state: State, client: KortexClient) -> None:
        self.state = state
        self.client = client
        self.log = lambda msg: print(msg)
        self.ratios = {}

    def calibrate_all(self) -> None:
        """Calibrate all joints."""
        self.state.controller.calibrate = True

        # Joint 1, 2, 3, 4:
        for n in [1, 2, 3, 4]:
            self.calibrate_joint_with_gravity(n)

        # Joint 0
        avg_medium_joint_ratio = self.ratios[2]
        self.calibrate_joint_with_given_ratio(0, avg_medium_joint_ratio)

        # Joint 5:
        small_joint_ratios = [self.ratios[3], self.ratios[4]]
        avg_small_joint_ratio = sum(small_joint_ratios) / len(small_joint_ratios)
        self.calibrate_joint_with_given_ratio(5, avg_small_joint_ratio)

        self.state.controller.calibrate = False

    def calibrate_joint_with_given_ratio(self, joint: int, ratio: float) -> None:
        """Determine the dynamic friction of a joint, given the ratio."""
        start, end = self.define_start_and_end(joint)

        frictions = []
        for _ in range(2):
            self.client._high_level_move(Position("", start))
            time.sleep(1)
            self.record_data(joint, start, end)

            result = minimize(self.f_move, 0)
            frictions.append(abs(result.x[0]))
            start, end = end, start
        friction = (sum(frictions) / len(frictions)) / ratio
        self.log(f"\nJoint {joint}: {(ratio):.2f} {friction:.2f}")

    def calibrate_joint_with_gravity(self, joint: int) -> None:
        """Determine the ratio and dynamic friction using gravity."""
        start, end = self.define_start_and_end(joint)

        ratios = []
        frictions = []
        for _ in range(2):
            self.client._high_level_move(Position("", start))
            time.sleep(1)
            self.record_data(joint, start, end)

            result = minimize(self.f_scale_and_move, [1, 0])
            ratios.append(result.x[0])
            frictions.append(result.x[1])
            start, end = end, start

        ratio = 1 / (sum(ratios) / len(ratios))
        self.ratios[joint] = ratio
        friction = abs(sum(frictions))
        self.log(f"\nJoint {joint}: {(ratio):.2f} {friction:.2f}")

    def define_start_and_end(self, joint: int) -> tuple[list, list]:
        """Define the start and end position."""
        start = [0 if joint != n else -90 for n in range(self.client.actuator_count)]
        end = [0 if joint != n else 90 for n in range(self.client.actuator_count)]
        start[2] = 90 if joint == 3 else start[2]
        end[2] = 90 if joint == 3 else end[2]
        return start, end

    def record_data(self, joint: int, start: list, end: list) -> None:
        """Record the data."""
        velocity = []
        model_torque = []
        real_torque = []
        thread = Thread(target=self.client.high_level_move, args=[Position("", end)])
        thread.start()
        edge = abs(0.9 * start[joint])
        while abs(np.rad2deg(self.state.kinova_feedback.q[joint])) > edge:
            time.sleep(1 / self.client.frequency)
        while abs(np.rad2deg(self.state.kinova_feedback.q[joint])) < edge:
            velocity.append(self.state.kinova_feedback.dq[joint])
            model_torque.append((self.state.g[joint]))
            real_torque.append((self.client.get_current(joint, False)))
            time.sleep(1 / self.client.frequency)
        while self.client.mode == "high_level_moving":
            time.sleep(0.1)

        self.velocity = savgol_filter(velocity, 501, 2)
        self.model_torque = np.array(model_torque)
        self.real_torque = savgol_filter(real_torque, 501, 2)

    def f_scale_and_move(self, params: list) -> None:
        """Function that scales and moves."""
        a, b = params
        return np.sqrt(np.sum((self.model_torque - (a * self.real_torque + b)) ** 2))

    def f_move(self, b: float) -> None:
        """Function that scales."""
        return np.sqrt(np.sum((self.model_torque - (self.real_torque + b)) ** 2))

    def export_data(self, joint: int, ratio: float, friction: float) -> None:
        """Export the data."""
        data = {
            "Velocity": self.velocity,
            "Model torque": self.model_torque,
            "Real torque": self.real_torque,
            "Scaled": ratio * self.real_torque,
            "Transformed": ratio * self.real_torque + friction,
        }
        df = pd.DataFrame(data)
        pre = "simulate_" if self.client.simulate else ""
        directory = str(pkg_resources.files(calibration_data))
        df.to_csv(directory + f"/{pre}calibration_{joint}.csv", index=False)
