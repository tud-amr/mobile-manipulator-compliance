from typing import Literal
import numpy as np
from kinova.kortex_client import KortexClient
from .state import State


class Controller:
    """General controller template."""

    def __init__(self, client: KortexClient) -> None:
        self.state: State = client.state
        self.client = client
        self.mode: Literal["position", "velocity", "current"] = "current"

    def connect_to_LLC(self) -> None:
        """Connect self to LLC controller."""
        self.joints = [n for n, active in enumerate(self.state.active) if active]
        self.client.connect_LLC(self, self.mode)

    def command(self) -> None:
        """Update the command of the robot."""
        if self.mode == "current":
            self.commands = np.zeros(len(self.joints))


class CompensateGravity(Controller):
    """Compensate gravity."""

    def command(self) -> None:
        """Compensate gravity."""
        super().command()
        for n, joint in enumerate(self.joints):
            self.commands[n] += (
                self.state.g[joint] * self.state.current_torque_ratios[joint]
            )


class Impedance(CompensateGravity):
    """Impedance control."""

    def __init__(self, client: KortexClient) -> None:
        super().__init__(client)
        if self.client.mock:
            self.S = [8, 8, 8, 2.00, 2.00, 0.0100]
            self.D = [2, 2, 2, 0.25, 0.25, 0.0025]
        else:
            self.S = [4.0, 6, 4.0, 2.00, 2.00, 1]
            self.D = [0.5, 0, 0.5, 0.25, 0.25, 0]

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

        for n, joint in enumerate(self.joints):
            self.commands[n] += tau[joint]


class CartesianImpedance(CompensateGravity):
    """Cartesian impedance control."""

    def __init__(self, client: KortexClient) -> None:
        super().__init__(client)
        if client.mock:
            self.Kd = np.eye(3) * 100
            self.Dd = np.eye(3) * 50
        else:
            self.Kd = np.eye(3) * 120
            self.Dd = np.eye(3) * 5

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

        for n, joint in enumerate(self.joints):
            self.commands[n] += tau[joint]


class TargetMover:
    """Used to move the target."""

    def __init__(self, state: State, client: KortexClient) -> None:
        self.state = state
        self.target_rate = 0.1  # m/s
        self.target_step = self.target_rate / client.frequency
        self.target_rot = 90
        self.target_rot_rate = 60  # deg / s
        self.target_rot_step = self.target_rot_rate / client.frequency

    def move(self) -> None:
        """Move the target."""
        self.target_rot += self.target_rot_step
        step_x = np.cos(np.deg2rad(self.target_rot)) * self.target_step
        step_y = np.sin(np.deg2rad(self.target_rot)) * self.target_step
        new_target_pos = self.state.target + np.array([step_x, step_y, 0])
        self.state.update_marker("target", new_target_pos)
