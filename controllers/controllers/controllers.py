from typing import Literal
import numpy as np
from .state import State


class Controller:
    """General controller template."""

    friction_compensation = False

    @staticmethod
    def toggle_CF() -> None:
        """Toggle compensate friction."""
        Controller.friction_compensation = not Controller.friction_compensation

    @staticmethod
    def get_CF() -> bool:
        """Return the state of friction compensation."""
        return Controller.friction_compensation

    @property
    def joints(self) -> list:
        return [n for n, active in enumerate(self.state.active) if active]

    def __init__(self, state: State) -> None:
        self.state = state
        self.mode: Literal["position", "velocity", "current"] = "current"

    def command(self) -> None:
        """Update the command of the robot."""
        if self.mode == "current":
            self.commands = np.zeros(len(self.joints))


class CompensateGravity(Controller):
    """Compensate gravity."""

    def __init__(self, state: State) -> None:
        super().__init__(state)

    def command(self) -> None:
        """Compensate gravity."""
        super().command()
        for n, joint in enumerate(self.joints):
            self.commands[n] += (
                self.state.g[joint] * self.state.current_torque_ratios[joint]
            )


class CompensateGravityAndFriction(CompensateGravity):
    """Compensate gravity and friction."""

    def __init__(self, state: State) -> None:
        super().__init__(state)
        self.friction_threshold = 0.1

    def command(self) -> None:
        """Compensate friction."""
        super().command()
        if not Controller.friction_compensation:
            return
        for n, joint in enumerate(self.joints):
            vel = self.state.dq[joint]
            abs_vel = abs(vel)
            if abs_vel > 0:
                comp = vel / abs_vel * self.state.dynamic_frictions[joint]
                comp *= (
                    abs_vel / self.friction_threshold
                    if abs_vel < self.friction_threshold
                    else 1
                )
                self.commands[n] += comp


class Impedance(CompensateGravity):
    """Impedance control."""

    def __init__(self, state: State) -> None:
        super().__init__(state)
        self.S = [4.0, 6, 4.0, 2.00, 2.00, 1]
        self.D = [0.5, 0.5, 0.5, 0.25, 0.25, 0]

        self.tolerance = 0.01  # m
        self.thr_dynamic = 0.3  # m/s

    def connect_to_LLC(self) -> None:
        """Set current q and dq as desired."""
        self.q_d = self.state.q.copy()
        self.dq_d = self.state.dq.copy()
        self.state.update_marker("target", self.state.x)
        super().connect_to_LLC()

    def command(self) -> None:
        """Joint space impedance control."""
        super().command()
        q_e = self.q_d - self.state.q
        dq_e = self.dq_d - self.state.dq
        tau = self.S * q_e + self.D * dq_e
        current = [tau[n] * self.state.get_ratio(n) for n in range(len(tau))]

        if Controller.friction_compensation:
            x = self.state.x
            x_d = self.state.target
            x_e = x_d - x
            self.compensate_friction(current, x_e)

        for n, joint in enumerate(self.joints):
            self.commands[n] += current[joint]

    def compensate_friction(self, current: np.ndarray, x_e: np.ndarray) -> np.ndarray:
        """Compensate friction."""
        for n in range(len(current)):
            if current[n] != 0 and np.linalg.norm(x_e) > self.tolerance:
                factor = min(abs(self.state.dq[n]) / self.thr_dynamic, 1)
                sign = current[n] / abs(current[n])
                static_part = (1 - factor) * self.state.static_frictions[n]
                dynamic_part = factor * self.state.dynamic_frictions[n]
                current[n] += sign * (static_part + dynamic_part)


class CartesianImpedance(CompensateGravity):
    """Cartesian impedance control."""

    def __init__(self, state: State) -> None:
        super().__init__(state)
        self.Kd = np.eye(3) * 25
        self.Dd = np.eye(3) * 0.5

        self.tolerance = 0.01  # m
        self.thr_dynamic = 0.3  # m/s

    def connect_to_LLC(self) -> None:
        """Set target to current position."""
        self.state.active = [True, False, True, False, True, False]
        self.state.update_marker("target", self.state.x)
        self.target_mover = TargetMover(self.state, self.client)
        super().connect_to_LLC()

    def command(self) -> None:
        """Passivity based method."""
        super().command()
        self.target_mover.move()

        Minv = np.linalg.inv(self.state.M)
        Jinv = (
            Minv @ self.state.J.T @ np.linalg.inv(self.state.J @ Minv @ self.state.J.T)
        )

        x = self.state.x
        dx = self.state.J[:3] @ self.state.dq

        x_d = self.state.target
        dx_d = np.zeros(3)
        ddx_d = np.zeros(3)

        x_e = x_d - x
        dx_e = dx_d - dx

        lam = Jinv.T @ self.state.M @ Jinv
        mu = Jinv.T @ (self.state.C - self.state.M @ Jinv @ self.state.dJ) @ Jinv

        f = lam @ ddx_d + mu @ dx_d + (self.Kd @ x_e + self.Dd @ dx_e)
        tau = self.state.J.T @ f
        current = [tau[n] * self.state.get_ratio(n) for n in range(len(tau))]

        if Controller.friction_compensation:
            self.compensate_friction(current, x_e)

        for n, joint in enumerate(self.joints):
            self.commands[n] += current[joint]

    def compensate_friction(self, current: np.ndarray, x_e: np.ndarray) -> np.ndarray:
        """Compensate friction."""
        for n in range(len(current)):
            if current[n] != 0 and np.linalg.norm(x_e) > self.tolerance:
                factor = min(abs(self.state.dq[n]) / self.thr_dynamic, 1)
                sign = current[n] / abs(current[n])
                static_part = (1 - factor) * self.state.static_frictions[n]
                dynamic_part = factor * self.state.dynamic_frictions[n]
                current[n] += sign * (static_part + dynamic_part)


class TargetMover:
    """Used to move the target."""

    def __init__(self, state: State) -> None:
        self.state = state
        self.target_rate = 0.05  # m/s
        self.target_step = self.target_rate / client.frequency
        self.target_rot = 90
        self.target_rot_rate = 30  # deg / s
        self.target_rot_step = self.target_rot_rate / client.frequency

    def move(self) -> None:
        """Move the target."""
        self.target_rot += self.target_rot_step
        step_x = np.cos(np.deg2rad(self.target_rot)) * self.target_step
        step_y = np.sin(np.deg2rad(self.target_rot)) * self.target_step
        new_target_pos = self.state.target + np.array([step_x, step_y, 0])
        self.state.update_marker("target", new_target_pos)
