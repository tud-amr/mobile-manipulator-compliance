import numpy as np

from scipy.signal import savgol_filter
from scipy.optimize import minimize

from compliant_control.kinova.kortex_client import KortexClient
from compliant_control.kinova.specifications import Position
from compliant_control.control.state import State

import time
from threading import Thread

import importlib.resources as pkg_resources
import casadi
import pinocchio
import compliant_control.mujoco.models as models

JOINTS = 6


class Calibration:
    """Contains all the calibrations."""

    def __init__(self, state: State, client: KortexClient) -> None:
        self.state = state
        self.client = client
        self.log = lambda msg: print(msg)
        self.ratios = {}

        self.model_data: list[np.ndarray] = []
        self.real_data: list[np.ndarray] = []
        self.theta: list[np.ndarray] = []

        self.load_model()

    def load_model(self) -> None:
        """Load the model."""
        urdf_package = str(pkg_resources.files(models))
        urdf = urdf_package + "/GEN3-LITE.urdf"
        self.robot = pinocchio.RobotWrapper.BuildFromURDF(urdf, urdf_package)
        self.robot.data = self.robot.model.createData()

    def calibrate_all(self) -> None:
        """Calibrate all joints."""
        self.state.controller.calibrate = True

        # Joint 4, 3, 2, 1:
        for joint in [4, 3, 2, 1]:
            start, end = self.define_start_and_end(joint)
            zero_pose = (np.deg2rad(start) + np.deg2rad(end)) / 2
            q_bf, theta_bf, torque_bf = self.record_back_and_forth(joint, start, end)
            lag = self.estimate_lag(theta_bf, torque_bf)
            flip = joint in [1, 2]
            axis = 0 if flip else 1
            link = joint + 1
            self.update_model_to_match_lag(joint, zero_pose, flip, link, lag, axis)
            ratio, friction = self.estimate_ratio_and_friction(joint, q_bf, torque_bf)
            self.ratios[joint] = ratio

        # Joint 3, Link 5Y:
        joint = 3
        start, end = self.define_start_and_end(joint)
        start[4] = end[4] = 90
        zero_pose = (np.deg2rad(start) + np.deg2rad(end)) / 2
        q_bf, theta_bf, torque_bf = self.record_back_and_forth(joint, start, end)
        lag = self.estimate_lag(theta_bf, torque_bf)
        link = 5
        flip = False
        axis = 0
        self.update_model_to_match_lag(joint, zero_pose, flip, link, lag, axis)

        # Joint 2, Link 5Z:
        joint = 2
        start, end = self.define_start_and_end(joint)
        start[3] = end[3] = 0
        zero_pose = (np.deg2rad(start) + np.deg2rad(end)) / 2
        q_bf, theta_bf, torque_bf = self.record_back_and_forth(joint, start, end)
        lag = self.estimate_lag(theta_bf, torque_bf)
        link = 5
        flip = True
        axis = 2
        self.update_model_to_match_lag(joint, zero_pose, flip, link, lag, axis)

        # Joint 2, Link 4Y:
        joint = 2
        start, end = self.define_start_and_end(joint)
        start[3] = end[3] = 0
        start[4] = end[4] = 90
        zero_pose = (np.deg2rad(start) + np.deg2rad(end)) / 2
        q_bf, theta_bf, torque_bf = self.record_back_and_forth(joint, start, end)
        lag = self.estimate_lag(theta_bf, torque_bf)
        link = 4
        flip = True
        axis = 0
        self.update_model_to_match_lag(joint, zero_pose, flip, link, lag, axis)

        # Joint 1, Link 4Z:
        joint = 1
        start, end = self.define_start_and_end(joint)
        start[3] = end[3] = 0
        start[4] = end[4] = 90
        zero_pose = (np.deg2rad(start) + np.deg2rad(end)) / 2
        q_bf, theta_bf, torque_bf = self.record_back_and_forth(joint, start, end)
        lag = self.estimate_lag(theta_bf, torque_bf)
        link = 4
        flip = True
        axis = 2
        self.update_model_to_match_lag(joint, zero_pose, flip, link, lag, axis)

        # # Joint 0, 5:
        # for joint in [0, 5]:
        #     if joint == 0:
        #         ratio = self.ratios[2]
        #     if joint == 5:
        #         ratio = (self.ratios[3] + self.ratios[4]) / 2
        #     start, end = self.define_start_and_end(joint)
        #     q_bf, theta_bf, torque_bf = self.record_back_and_forth(joint, start, end)
        #     self.estimate_friction(joint, ratio, q_bf, torque_bf)

        self.state.controller.calibrate = False

    def define_start_and_end(self, joint: int) -> tuple[list, list]:
        """Define the start and end position of a calibration trajectory."""
        start = [0 if joint != n else -100 for n in range(self.client.actuator_count)]
        end = [0 if joint != n else 100 for n in range(self.client.actuator_count)]
        start[2] = 90 if joint == 3 else start[2]
        end[2] = 90 if joint == 3 else end[2]
        start[3] = 90 if joint in [1, 2] else start[3]
        end[3] = 90 if joint in [1, 2] else end[3]
        return start, end

    def record_back_and_forth(
        self, joint: int, start: list, end: list
    ) -> tuple[np.ndarray, ...]:
        """Record the joint positions and torques for the forward and backward trajectory."""
        q_bf = []
        theta_bf = []
        torque_bf = []
        l = float("inf")
        flip = False
        for _ in range(2):
            q, theta, torque = self.record_trajectory(joint, start, end, flip)
            if flip:
                q.reverse()
                theta.reverse()
                torque.reverse()
            q_bf.append(q)
            theta_bf.append(theta)
            torque_bf.append(torque)
            l = min(l, len(q), len(theta), len(torque))
            flip = not flip

        for n in range(2):
            q_bf[n] = np.array(q_bf[n][:l])
            theta_bf[n] = np.array(theta_bf[n][:l])
            torque_bf[n] = np.array(torque_bf[n][:l])

        return q_bf, theta_bf, torque_bf

    def estimate_lag(
        self, theta_bf: list[np.ndarray], torque_bf: list[np.ndarray]
    ) -> float:
        """Estimate the lag based on the recorded joint positions and torques."""
        self.log("\n")

        self.input = (
            savgol_filter(theta_bf[0], 501, 2) + savgol_filter(theta_bf[1], 501, 2)
        ) / 2
        self.output = (
            savgol_filter(torque_bf[0], 501, 2) + savgol_filter(torque_bf[1], 501, 2)
        ) / 2
        scale, lag = minimize(self.f_cos, [1, 0]).x

        while lag > np.pi / 2:
            self.log(f"Measured lag was outside domain: {lag}")
            lag -= np.pi
            self.log(f"Lag was changed to: {lag}")
        while lag < -np.pi / 2:
            self.log(f"Measured lag was outside domain: {lag}")
            lag += np.pi
            self.log(f"Lag was changed to: {lag}")

        return lag

    def update_model_to_match_lag(
        self, joint: int, pose: np.ndarray, flip: bool, link: int, lag: float, axis: int
    ) -> None:
        """Update the robot model to match the measured lag."""
        original_value = self.robot.model.inertias[link].lever[axis]

        for n in range(1, 10):
            step = 10**-n
            self.move_com_to_match_lag(joint, pose, flip, link, lag, axis, step)

        new_value = self.robot.model.inertias[link].lever[axis]
        self.log(f"link: {link}, original: {original_value}, new: {new_value}")

    def estimate_ratio_and_friction(
        self, joint: int, q_bf: list[np.ndarray], torque_bf: list[np.ndarray]
    ) -> tuple[float, ...]:
        """Estimate the ratio and friction of the joint."""
        avg_ratio = []
        avg_friction = []
        for q, torque in zip(q_bf, torque_bf):
            self.output = savgol_filter(torque, 501, 2)
            self.input = np.array([self.g(q_n, joint) for q_n in q])
            ratio, friction = minimize(self.f_scale_and_move, [1, 0]).x
            avg_ratio.append(ratio)
            avg_friction.append(friction)
            self.log(f"ratio: {ratio}, friction: {friction}")
        return np.average(avg_ratio), np.average(avg_friction)

    def estimate_friction(
        self,
        joint: int,
        ratio: float,
        q_bf: list[np.ndarray],
        torque_bf: list[np.ndarray],
    ) -> float:
        """Estimate the friction of a joint, given the ratio."""
        avg_friction = []
        for q, torque in zip(q_bf, torque_bf):
            self.input = ratio * np.array([self.g(q_n, joint) for q_n in q])
            self.output = savgol_filter(torque, 501, 2)
            friction = minimize(self.f_move, 0).x[0]
            avg_friction.append(friction)
            self.log(f"friction: {friction}")
        return np.average(avg_friction)

    def record_trajectory(
        self, joint: int, start: list, end: list, flip: bool = False
    ) -> tuple[list, ...]:
        """Record the joint position and torque during a single trajectory."""
        if flip:
            start, end = end, start
        self.client._high_level_move(Position("", start))
        q = []
        theta = []
        torque = []
        thread = Thread(target=self.client.high_level_move, args=[Position("", end)])
        thread.start()
        thr = 0.1 * abs(end[joint] - start[joint])
        while abs(start[joint] - np.rad2deg(self.state.kinova_feedback.q[joint])) < thr:
            time.sleep(1 / self.client.frequency)
        while abs(end[joint] - np.rad2deg(self.state.kinova_feedback.q[joint])) > thr:
            q.append(self.state.kinova_feedback.q.copy())
            theta.append(self.state.kinova_feedback.q[joint] + np.pi / 2)
            torque.append((self.client.get_current(joint, False)))
            time.sleep(1 / self.client.frequency)
        while self.client.mode == "high_level_moving":
            time.sleep(0.1)

        return q, theta, torque

    def move_com_to_match_lag(
        self,
        joint: int,
        pose: np.ndarray,
        flip: bool,
        link: int,
        lag: float,
        axis: int,
        step: float,
    ) -> None:
        """Move the center of mass over the given axis by the given step to match the measured lag."""
        model_lag = self.model_lag(joint, pose, flip)
        error = abs(model_lag - lag)
        self.log(f"step: {step}, model: {model_lag}, real: {lag}, error: {error}")
        improve = True
        while improve:
            self.robot.model.inertias[link].lever[axis] += step
            new_error = abs(self.model_lag(joint, pose, flip) - lag)
            if new_error <= error:
                error = new_error
                continue
            self.robot.model.inertias[link].lever[axis] -= step
            if step > 0:
                step *= -1
            else:
                improve = False

    def model_lag(self, joint: int, pose: np.ndarray, flip: bool) -> float:
        """Return the lag based on the model."""
        op, ad = [0, 1] if flip else [1, 0]
        pinocchio.centerOfMass(self.robot.model, self.robot.data, pose, True)
        lag = np.arctan(
            self.robot.data.com[joint + 1][op] / self.robot.data.com[joint + 1][ad]
        )
        return -lag if flip else lag

    def f_scale_and_move(self, params: list) -> None:
        """Function that scales and moves."""
        a, b = params
        return np.sqrt(np.sum((self.output - (a * self.input + b)) ** 2))

    def f_move(self, b: float) -> None:
        """Function that moves."""
        return np.sqrt(np.sum((self.output - (self.input + b)) ** 2))

    def f_cos(self, params: list) -> None:
        """Function that multiplies with cosine."""
        scale, lag = params
        return np.sqrt(np.sum((self.output - (scale * np.cos(self.input + lag))) ** 2))

    def g(self, q: np.ndarray, joint: int) -> float:
        """Get the model_torque based on gravity."""
        return pinocchio.computeGeneralizedGravity(
            self.robot.model, self.robot.data, q
        )[joint]
