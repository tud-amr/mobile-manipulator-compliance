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
RATE = 1000


class Calibration:
    """Contains all the calibrations."""

    def __init__(self, state: State, client: KortexClient, publish: callable) -> None:
        self.state = state
        self.client = client
        self.publish = publish
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

    def calibrate_lag(self) -> list[float]:
        """Define the lag for a rotation, correct the model and return the new position."""
        f, t = False, True

        jnts = [4, 3, 2, 1, 3, 2, 2]
        flps = [f, f, t, t, f, t, t]
        lnks = [5, 4, 3, 2, 5, 5, 4]
        axes = [1, 1, 0, 0, 0, 2, 0]
        chg2 = [f, t, f, f, t, f, f]
        chg3 = [f, f, t, t, f, f, f]
        chg4 = [f, f, f, f, t, f, t]

        # Default pose:
        values = []
        for joint, flip, link, axis, change2, change3, change4 in zip(
            jnts, flps, lnks, axes, chg2, chg3, chg4
        ):
            start, end = self.define_start_and_end(joint)
            if change2:
                start[2] = end[2] = 90
            if change3:
                start[3] = end[3] = 90
            if change4:
                start[4] = end[4] = 90
            zero_pose = (np.deg2rad(start) + np.deg2rad(end)) / 2

            q_bf, theta_bf, torque_bf = self.record_back_and_forth(joint, start, end)
            lag = self.estimate_lag(theta_bf, torque_bf)
            value = self.new_value_to_match_lag(joint, zero_pose, flip, link, lag, axis)
            values.append(value)
        return values

    def calibrate_ratio_and_friction(self) -> tuple[float, ...]:
        """Calibrate the ratio and friction."""
        f, t = False, True

        jnts = [1]
        chg2 = [f]
        chg3 = [t]

        # Joint 4, 3, 2, 1:
        values = []
        for joint, change2, change3 in zip(jnts, chg2, chg3):
            start, end = self.define_start_and_end(joint)
            if change2:
                start[2] = end[2] = 90
            if change3:
                start[3] = end[3] = 90
            q_bf, theta_bf, torque_bf = self.record_back_and_forth(joint, start, end)
            ratio, friction = self.estimate_ratio_and_friction(joint, q_bf, torque_bf)
            values.append(ratio)
            values.append(friction)
        return values

    def calibrate_friction(self) -> tuple[float, ...]:
        """Calibrate the friction."""
        jnts = [0, 5]
        values = []
        for joint in jnts:
            start, end = self.define_start_and_end(joint)
            q_bf, theta_bf, torque_bf = self.record_back_and_forth(joint, start, end)
            friction = self.estimate_friction(
                joint, self.state.ratios[joint], q_bf, torque_bf
            )
            values.append(friction)
        return values

    def calibrate_all(self) -> None:
        """Calibrate all joints."""
        self.state.controller.calibrate = True

        N = 1
        start = time.time()

        for n in range(N):
            self.log(f"ROUND {n}:")
            values = self.calibrate_lag()
            self.publish(values)
            self.log(f"Round finished, total running time: {time.time() - start}")

        for n in range(N):
            self.log(f"ROUND {n}:")
            values = self.calibrate_ratio_and_friction()
            self.publish(values)
            self.log(f"Round finished, total running time: {time.time() - start}")

        for n in range(N):
            self.log(f"ROUND {n}:")
            values = self.calibrate_friction()
            self.publish(values)
            self.log(f"Round finished, total running time: {time.time() - start}")

        self.state.controller.calibrate = False

    def define_start_and_end(self, joint: int) -> tuple[list, list]:
        """Define the start and end position of a calibration trajectory."""
        start = [0 if joint != n else -100 for n in range(self.client.actuator_count)]
        end = [0 if joint != n else 100 for n in range(self.client.actuator_count)]
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

    def new_value_to_match_lag(
        self, joint: int, pose: np.ndarray, flip: bool, link: int, lag: float, axis: int
    ) -> float:
        """Update the robot model to match the measured lag and return the new value."""
        original_value = self.robot.model.inertias[link].lever[axis]

        for n in range(1, 10):
            step = 10**-n
            self.move_com_to_match_lag(joint, pose, flip, link, lag, axis, step)

        new_value = self.robot.model.inertias[link].lever[axis]
        self.log(f"link: {link}, axis: {axis}, old: {original_value}, new: {new_value}")
        return new_value

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
            avg_friction.append(abs(friction))
            self.log(f"joint: {joint}, ratio: {ratio}, friction: {friction}")
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
            avg_friction.append(abs(friction))
            self.log(f"joint: {joint}, ratio: {ratio}, friction: {friction}")
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
            time.sleep(1 / RATE)
        while abs(end[joint] - np.rad2deg(self.state.kinova_feedback.q[joint])) > thr:
            q.append(self.state.kinova_feedback.q.copy())
            theta.append(self.state.kinova_feedback.q[joint] + np.pi / 2)
            torque.append((self.client.get_current(joint, False)))
            time.sleep(1 / RATE)
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
