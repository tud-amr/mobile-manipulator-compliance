from typing import Literal, TYPE_CHECKING
import numpy as np

from compliant_control.dingo.utilities import direction_to_wheel_torques

if TYPE_CHECKING:
    from compliant_control.control.state import State

JOINTS = 6
WHEELS = 4


class Controller:
    """General controller template."""

    def __init__(self, state: "State") -> None:
        self.state = state
        self.comp_grav = True
        self.comp_fric = False
        self.imp_joint = False
        self.imp_cart = False
        self.calibrate = False
        self.mode: Literal["position", "velocity", "current"] = "current"

        self.calibration_addition = np.zeros(JOINTS)

        # Joint impedance:
        self.S = [4.0, 6, 4.0, 2.00, 2.00, 1]
        self.D = [0.5, 0.5, 0.5, 0.25, 0.25, 0]

        # Cartesian impedance:
        self.Kd = np.eye(3) * 25
        self.Dd = np.eye(3) * 0.5

        # General:
        self.tolerance = 0.01  # m
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
        self.q_d = self.state.kinova_feedback.q.copy()
        self.dq_d = self.state.kinova_feedback.dq.copy()

        self.x_d = self.state.x.copy()
        self.dx_d = np.zeros(3)
        self.ddx_d = np.zeros(3)

        self.pref_x = self.state.x.copy()

    def define_errors(self) -> None:
        """Define the errors."""
        x = self.state.x
        dx = self.state.J @ self.state.kinova_feedback.dq

        self.x_e = self.x_d - x
        self.dx_e = self.dx_d - dx

    def command(self) -> None:
        """Update the command of the robot."""
        self.joint_commands = np.zeros(JOINTS)
        self.x_d = self.state.target
        self.define_errors()
        if self.comp_grav:
            self.compensate_gravity()
        if self.imp_joint:
            self.joint_impedance()
        if self.imp_cart:
            self.cartesian_impedance()
            self.command_base()
        if self.comp_fric and not (self.imp_joint or self.imp_cart):
            self.compensate_friction_when_moving()
        if self.calibrate:
            self.calibration_value()

    def compensate_gravity(self) -> None:
        """Compensate gravity."""
        self.joint_commands += self.state.g * self.state.ratios

    def joint_impedance(self) -> None:
        """Joint impedance."""
        q_e = self.q_d - self.state.kinova_feedback.q
        dq_e = self.dq_d - self.state.kinova_feedback.dq
        tau = self.S * q_e + self.D * dq_e
        current = tau * self.state.ratios

        if self.comp_fric:
            current = self.compensate_friction(current)
        self.joint_commands += np.array(current)

    def cartesian_impedance(self) -> None:
        """Cartesian impedance."""
        f = (
            self.state.lam @ self.ddx_d
            + self.state.mu @ self.dx_d
            + (self.Kd @ self.x_e + self.Dd @ self.dx_e)
        )
        tau = self.state.JT @ f
        current = np.reshape(tau * self.state.ratios, JOINTS)

        if self.comp_fric:
            current = self.compensate_friction(current)
        self.joint_commands += np.array(current)

    def compensate_friction(self, current: np.ndarray) -> np.ndarray:
        """Compensate friction."""
        for n in range(len(current)):
            if current[n] != 0 and np.linalg.norm(self.x_e) > self.tolerance:
                sign = current[n] / abs(current[n])
                current[n] += sign * self.state.frictions[n]
        return current

    def compensate_friction_when_moving(self) -> None:
        """Compensate friction when moving."""
        for n in range(JOINTS):
            vel = self.state.kinova_feedback.dq[n]
            abs_vel = abs(vel)
            if abs_vel > 0:
                comp = vel / abs_vel * self.state.frictions[n]
                comp *= (
                    abs_vel / self.friction_threshold
                    if abs_vel < self.friction_threshold
                    else 1
                )
                self.joint_commands[n] += comp

    def command_base(self) -> None:
        """Create a command for the base."""
        error = (self.state.x - self.pref_x)[:-1]
        magnitude = np.linalg.norm(error)
        if magnitude > 0:
            direction = error / magnitude
            gain = min(magnitude / 0.15, 0.5)
            direction *= gain
            direction = [-direction[1], direction[0]]
        else:
            direction = [0.0, 0.0]
        self.command_base_direction(direction)

    def command_base_direction(self, direction: list[float], gain: float = 1) -> None:
        """Command the base with a direction."""
        self.state.dingo_command.c = direction_to_wheel_torques(direction) * gain

    def calibration_value(self) -> None:
        """Increase torque until joint moves."""
        self.joint_commands += self.calibration_addition
