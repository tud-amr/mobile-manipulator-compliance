from typing import Literal
import numpy as np
from .state import State


class Controller:
    """General controller template."""

    def __init__(self, state: State) -> None:
        self.state = state
        self.active = False
        self.comp_grav = True
        self.comp_fric = False
        self.imp_joint = False
        self.imp_cart = False
        self.mode: Literal["position", "velocity", "current"] = "current"

        # Joint impedance:
        self.S = [4.0, 6, 4.0, 2.00, 2.00, 1]
        self.D = [0.5, 0.5, 0.5, 0.25, 0.25, 0]

        # Cartesian impedance:
        self.Kd = np.eye(3) * 25
        self.Dd = np.eye(3) * 0.5

        # General:
        self.tolerance = 0.01  # m
        self.thr_dynamic = 0.3  # m/s
        self.friction_threshold = 0.1

    def toggle(self, name: str) -> None:
        """Toggle controller states."""
        match name:
            case "gravity":
                self.comp_grav = not self.comp_grav
            case "friction":
                self.comp_fric = not self.comp_fric
            case "joint":
                self.imp_joint = not self.imp_joint
                self.imp_cart = False if self.imp_joint else self.imp_cart
            case "cartesian":
                self.imp_cart = not self.imp_cart
                self.imp_joint = False if self.imp_cart else self.imp_joint
        self.reset()

    def reset(self) -> None:
        """Reset self to prepare for connection."""
        self.q_d = self.state.q.copy()
        self.dq_d = self.state.dq.copy()

        self.x_d = self.state.x.copy()
        self.dx_d = np.zeros(3)
        self.ddx_d = np.zeros(3)

    def define_errors(self) -> None:
        """Define the errors."""
        x = self.state.x
        dx = self.state.J @ self.state.dq

        self.x_e = self.x_d - x
        self.dx_e = self.dx_d - dx

    def command(self) -> None:
        """Update the command of the robot."""
        self.commands = np.zeros(6)
        self.x_d = self.state.target
        self.define_errors()
        if self.comp_grav:
            self.compensate_gravity()
        if self.imp_joint:
            self.joint_impedance()
        if self.imp_cart:
            self.cartesian_impedance()

    def compensate_gravity(self) -> None:
        """Compensate gravity."""
        self.commands += self.state.g * self.state.ratios

    def joint_impedance(self) -> None:
        """Joint impedance."""
        q_e = self.q_d - self.state.q
        dq_e = self.dq_d - self.state.dq
        tau = self.S * q_e + self.D * dq_e
        current = tau * self.state.ratios

        if self.comp_fric:
            current = self.compensate_friction(current)
        self.commands += np.array(current)

    def cartesian_impedance(self) -> None:
        """Cartesian impedance."""
        f = (
            self.state.lam @ self.ddx_d
            + self.state.mu @ self.dx_d
            + (self.Kd @ self.x_e + self.Dd @ self.dx_e)
        )
        tau = self.state.JT @ f
        current = np.reshape(tau * self.state.ratios, 6)

        if self.comp_fric:
            current = self.compensate_friction(current)
        self.commands += np.array(current)

    def compensate_friction(self, current: np.ndarray) -> np.ndarray:
        """Compensate friction."""
        for n in range(len(current)):
            if current[n] != 0 and np.linalg.norm(self.x_e) > self.tolerance:
                factor = min(abs(self.state.dq[n]) / self.thr_dynamic, 1)
                sign = current[n] / abs(current[n])
                static_part = (1 - factor) * self.state.static_frictions[n]
                dynamic_part = factor * self.state.dynamic_frictions[n]
                current[n] += sign * (static_part + dynamic_part)
        return current

    def compensate_friction_when_moving(self) -> None:
        """Compensate friction when moving."""
        for n, command in enumerate(self.commands):
            vel = self.state.dq[n]
            abs_vel = abs(vel)
            if abs_vel > 0:
                comp = vel / abs_vel * self.state.dynamic_frictions[n]
                comp *= (
                    abs_vel / self.friction_threshold
                    if abs_vel < self.friction_threshold
                    else 1
                )
                command += comp
        self.commands[n] = command
