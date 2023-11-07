import numpy as np
import pandas as pd
import importlib.resources as pkg_resources
from typing import Literal

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

        self.tolerance = 0.01
        self.increase = 0.001

    def calibrate_all(self, mode: Literal["static", "dynamic"]) -> None:
        """Calibrate all joints."""
        self.state.controller.calibrate = True
        self.client.joint_active = [True] * self.client.actuator_count
        if mode == "static":
            self.static_friction(0, [0, 0, 0, 0, 0, 0])
            self.static_friction(1, [0, 0, 0, 90, 0, 0])
            self.static_friction(2, [0, 0, 0, 90, 0, 0])
            self.static_friction(3, [0, 0, 0, 0, 0, 0])
            self.static_friction(4, [0, 0, 90, 0, 0, 0])
            self.static_friction(5, [0, 0, 0, 0, 0, 0])
        if mode == "dynamic":
            self.ratio_and_dynamic_friction(0)
            self.ratio_and_dynamic_friction(1)
            self.ratio_and_dynamic_friction(2)
            self.ratio_and_dynamic_friction(3)
            self.ratio_and_dynamic_friction(4)
            self.ratio_and_dynamic_friction(5)
        self.state.controller.calibrate = False

    def static_friction(self, joint: int, start_pos: list) -> None:
        """Determine the static friction of the joint."""
        position = start_pos
        angles = np.linspace(-45, 45, 1)
        frictions = []
        for angle in angles:
            position[joint] = angle
            self.state.controller.calibration_addition = np.zeros(JOINTS)
            self.client._high_level_move(Position("", position))
            self.client.start_LLC()
            self.client.connect_LLC()
            time.sleep(1)
            moved = False
            self.state.controller.calibrate = True
            while not moved:
                moved = abs(self.state.kinova_feedback.dq[joint]) > self.tolerance
                self.state.controller.calibration_addition[joint] += 0.001
                time.sleep(0.01)
            self.state.controller.calibrate = False
            self.client.disconnect_LLC()
            self.client.stop_LLC()
            frictions.append(self.state.controller.calibration_addition[joint])
        self.log(str(frictions))

    def ratio_and_dynamic_friction(self, joint: int) -> None:
        """Determine the ratio and dynamic friction."""
        while self.client.mode == "high_level_moving":
            time.sleep(0.1)
        start = [0 if joint != n else -90 for n in range(self.client.actuator_count)]
        end = [0 if joint != n else 90 for n in range(self.client.actuator_count)]
        start[2] = 90 if joint == 3 else start[2]
        end[2] = 90 if joint == 3 else end[2]
        self.client._high_level_move(Position("", start))
        time.sleep(1)

        self.record_data(joint, end)

        if sum(abs(self.model_torque)) > 1:
            result = minimize(self.f_scale_and_move, [1, 0])
            ratio = result.x[0]
            friction = result.x[1]
        else:
            result = minimize(self.f_move, 0)
            ratio = 1
            friction = result.x[0]

        self.log(f"Joint {joint}: {1/ratio, friction}")
        self.export_data(joint, ratio, friction)

    def record_data(self, joint: int, end: list) -> None:
        """Record the data."""
        velocity = []
        model_torque = []
        real_torque = []
        thread = Thread(target=self.client.high_level_move, args=[Position("", end)])
        thread.start()
        while np.rad2deg(self.state.kinova_feedback.q[joint]) < -70:
            time.sleep(1 / self.client.frequency)
        while np.rad2deg(self.state.kinova_feedback.q[joint]) < 70:
            velocity.append(self.state.kinova_feedback.dq[joint])
            model_torque.append((self.state.g[joint]))
            real_torque.append((self.client.get_current(joint, False)))
            time.sleep(1 / self.client.frequency)

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
