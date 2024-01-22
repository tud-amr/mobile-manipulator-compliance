from __future__ import annotations
from typing import Literal, TYPE_CHECKING
import numpy as np
from threading import Thread

from compliant_control.dingo.utilities import (
    direction_to_wheel_torques,
    rotation_to_wheel_torques,
)
from compliant_control.utilities.rate_counter import RateCounter
from compliant_control.kinova.specifications import Position

if TYPE_CHECKING:
    from compliant_control.control.state import State

JOINTS = 6
WHEELS = 4


class Controller:
    """General controller template."""

    def __init__(self, state: "State") -> None:
        self.state = state
        self.active = False
        self.comp_grav = True
        self.comp_fric = False
        self.imp_arm = False
        self.imp_null = False
        self.imp_base = False
        self.mode: Literal["position", "velocity", "current"] = "current"

        self.joint_commands = np.zeros(JOINTS)
        self.c_compliant = np.zeros(JOINTS)
        self.c_nullspace = np.zeros(JOINTS)
        self.c_compensate = np.zeros(JOINTS)
        self.rate_counter = RateCounter(1000)

        # Cartesian impedance:
        self.thr_cart_error = 0.001  # m
        self.Kd = np.eye(3) * 40
        self.Dd = np.eye(3) * 3
        self.error_cart_MAX = 0.1  # m

        # Null space:
        self.K_n = np.eye(6) * 0.2
        self.D_n = np.eye(6) * 0.1

        # Base
        self.thr_pos_error = 0.01  # m
        self.thr_rot_error = np.deg2rad(10)
        self.K_pos = 3
        self.gain_pos_MAX = 1
        self.K_rot = 1
        self.gain_rot_MAX = 0.3

        # General:
        self.thr_dynamic = 0.15  # rad/s
        self.fac_joint1 = 0.6

    def toggle(self, name: str) -> None:
        """Toggle controller states."""
        if name == "grav":
            self.comp_grav = not self.comp_grav
        elif name == "fric":
            self.comp_fric = not self.comp_fric
        elif name == "arm":
            self.imp_arm = not self.imp_arm
        elif name == "null":
            self.imp_null = not self.imp_null
        elif name == "base":
            self.pref_x = self.state.x.copy()
            self.imp_base = not self.imp_base
        self.reset()

    def reset(self) -> None:
        """Reset self to prepare for connection."""
        self.dx_d = np.zeros(3)
        self.ddx_d = np.zeros(3)

        self.pref_x = self.state.x.copy()

    def update_command(self) -> None:
        """Update the command of the robot."""
        current = np.zeros(JOINTS)
        if self.imp_arm:
            current += self.cartesian_impedance()
            self.c_compliant = current.copy()
            if self.imp_null:
                current += self.null_space_task()
                self.c_nullspace = current.copy()
            if self.comp_fric:
                current += self.compensate_friction_in_impedance_mode(current)
                self.c_compensate = current.copy()
            if self.imp_base:
                self.command_base()
        else:
            if self.comp_fric:
                current += self.compensate_friction_in_moving_direction()
        if self.comp_grav:
            current += self.compensate_gravity()
            self.joint_commands = current

    def compensate_gravity(self) -> np.ndarray:
        """Return the current due to gravity compensation."""
        return self.state.g * self.state.ratios

    def cartesian_impedance(self) -> np.ndarray:
        """Return the current due to cartesian impedance.."""
        self.x_e = np.zeros(3)
        error = self.state.target - self.state.x
        magnitude = np.linalg.norm(error)
        if magnitude > self.thr_cart_error:
            vector = error / magnitude
            self.x_e = vector * min(self.error_cart_MAX, magnitude)
        self.dx_e = self.dx_d - self.state.dx
        force = self.Kd @ self.x_e + self.Dd @ self.dx_e
        torque = self.state.T(force)
        return torque * self.state.ratios

    def null_space_task(self) -> None:
        """Return the current due to the null space task."""
        qd = np.deg2rad(Position.pref.position)
        torque = (
            self.K_n @ (qd - self.state.kinova_feedback.q)
            - self.D_n @ self.state.kinova_feedback.dq
        )
        current = torque * self.state.ratios
        return self.state.N @ current

    def compensate_friction_in_impedance_mode(self, current: np.ndarray) -> np.ndarray:
        """Add friction compensation while using impedance mode."""
        comp_dir_mov = self.compensate_friction_in_moving_direction()
        comp_dir_cur = self.compensate_friction_in_current_direction(current)
        compensation = comp_dir_mov + comp_dir_cur
        # Decrease compensation when close to target:
        compensation *= min(np.linalg.norm(self.x_e) / self.thr_pos_error, 1)
        # Reduce compensation for second joint, because of high inertia:
        compensation[1] *= self.fac_joint1
        return compensation

    def compensate_friction_in_current_direction(
        self, current: np.ndarray
    ) -> np.ndarray:
        """Add friction compensation to the given current, when not moving."""
        dq = self.state.kinova_feedback.dq
        frac_v = 1 - np.minimum(np.abs(dq) / self.thr_dynamic, 1)
        return frac_v * np.sign(current) * self.state.frictions

    def compensate_friction_in_moving_direction(self) -> np.ndarray:
        """Add friction compensation in the direction the joint is moving."""
        dq = self.state.kinova_feedback.dq
        frac_v = np.minimum(np.abs(dq) / self.thr_dynamic, 1)
        return frac_v * np.sign(dq) * self.state.frictions

    def command_base(self) -> None:
        """Create a command for the base."""
        self.reset_base_command()
        
        # Position:
        error = (self.state.x - self.pref_x)[:-1]
        magnitude = np.linalg.norm(error)
        if magnitude > self.thr_pos_error:
            direction = error / magnitude
            gain = min(magnitude * self.K_pos, self.gain_pos_MAX)
            self.command_base_direction([-direction[1], direction[0]], gain)

        # Rotation:
        error = -self.state.kinova_feedback.q[0]
        magnitude = abs(error)
        if magnitude > self.thr_rot_error:
            rotation = error
            gain = min(magnitude * self.K_rot, self.gain_rot_MAX)
            self.command_base_rotation(rotation, gain)

    def reset_base_command(self) -> None:
        """Reset the base command."""
        self.state.dingo_command.c = np.zeros(WHEELS)

    def command_base_direction(self, direction: list[float], gain: float = 1) -> None:
        """Command the base with a direction."""
        self.state.dingo_command.c += direction_to_wheel_torques(direction) * gain

    def command_base_rotation(self, rotation: float, gain: float = 1) -> None:
        """Command the base with a rotation."""
        self.state.dingo_command.c += rotation_to_wheel_torques(rotation) * gain

    def start_control_loop(self) -> None:
        """Start the control loop."""
        self.active = True
        thread = Thread(target=self.control_loop)
        thread.start()

    def stop_control_loop(self) -> None:
        """Stop the control loop."""
        self.active = False
        self.state.dingo_command.c = np.zeros(WHEELS)

    def control_loop(self) -> None:
        """Control loop."""
        while self.active:
            self.update_command()
            self.rate_counter.count()
            self.rate_counter.sleep()
