from typing import Literal, TYPE_CHECKING
import numpy as np
from threading import Thread

from compliant_control.dingo.utilities import direction_to_wheel_torques
from compliant_control.utilities.rate_counter import RateCounter

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
        self.imp_base = False
        self.mode: Literal["position", "velocity", "current"] = "current"

        self.joint_commands = np.zeros(JOINTS)
        self.rate_counter = RateCounter(1200)

        # Cartesian impedance:
        self.Kd = np.eye(3) * 25
        self.Dd = np.eye(3) * 0.5

        # Base
        self.percentage_max = 0.5
        self.K = 15

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
            case "arm":
                self.imp_arm = not self.imp_arm
            case "base":
                self.imp_base = not self.imp_base
        self.reset()

    def reset(self) -> None:
        """Reset self to prepare for connection."""
        self.dx_d = np.zeros(3)
        self.ddx_d = np.zeros(3)

        self.pref_x = self.state.x.copy()

    def update_command(self) -> None:
        """Update the command of the robot."""
        self.joint_commands = np.zeros(JOINTS)
        if self.comp_grav:
            self.compensate_gravity()

        if self.imp_arm:
            self.cartesian_impedance()
            if self.imp_base:
                self.command_base()
        else:
            if self.comp_fric:
                self.compensate_friction_when_moving()

    def compensate_gravity(self) -> None:
        """Compensate gravity."""
        self.joint_commands += self.state.g * self.state.ratios

    def cartesian_impedance(self) -> None:
        """Cartesian impedance."""
        self.x_e = self.state.target - self.state.x
        self.dx_e = self.dx_d - self.state.dx
        force = self.Kd @ self.x_e + self.Dd @ self.dx_e
        torque = self.state.T(force)
        current = torque * self.state.ratios

        if self.comp_fric and np.linalg.norm(self.x_e) > self.tolerance:
            current += np.sign(current) * self.state.frictions
        self.joint_commands += np.array(current)

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
            gain = min(magnitude * self.K, self.percentage_max)
            direction *= gain
            direction = [-direction[1], direction[0]]
        else:
            direction = [0.0, 0.0]
        self.command_base_direction(direction)

    def command_base_direction(self, direction: list[float], gain: float = 1) -> None:
        """Command the base with a direction."""
        self.state.dingo_command.c = direction_to_wheel_torques(direction) * gain

    def start_control_loop(self) -> None:
        """Start the control loop."""
        self.active = True
        thread = Thread(target=self.control_loop)
        thread.start()

    def stop_control_loop(self) -> None:
        """Stop the control loop."""
        self.active = False

    def control_loop(self) -> None:
        """Control loop."""
        while self.active:
            self.update_command()
            self.rate_counter.count()
            self.rate_counter.sleep()
