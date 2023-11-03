import mujoco
from mujoco import mj_name2id, mjtObj
import numpy as np
from typing import Literal
from compliant_control.mujoco.visualization import Visualization
from compliant_control.kinova.specifications import Position
from compliant_control.control.state import State


class Simulation(Visualization):
    """Provides the mujoco simulation of the robot."""

    def __init__(self, state: State) -> None:
        super().__init__()
        self.state = state

        self.default_biasprm = self.model.actuator_biasprm.copy()
        self.default_gainprm = self.model.actuator_gainprm.copy()

        pref = [np.deg2rad(pos) for pos in Position.pref.position]
        self.set_ctrl_value("Kinova", "position", pref)
        self.set_qpos_value("Kinova", "position", pref)

    def step(self) -> None:
        """Perform a simulation step."""
        mujoco.mj_step(self.model, self.data)
        self.state.target = self.relative_target

    def change_mode(self, mode: Literal["position", "torque"], joint: int) -> None:
        """Change the control mode of the Kinova arm."""
        idx = mj_name2id(self.model, mjtObj.mjOBJ_ACTUATOR, f"Kinova_position_{joint}")
        if mode == "position":
            self.model.actuator_biasprm[idx][1] = self.default_biasprm[idx][1]
            self.model.actuator_gainprm[idx][0] = self.default_gainprm[idx][0]
        elif mode == "torque":
            self.model.actuator_biasprm[idx][1] = 0
            self.model.actuator_gainprm[idx][0] = 0

        idx = mj_name2id(self.model, mjtObj.mjOBJ_ACTUATOR, f"Kinova_velocity_{joint}")
        if mode == "position":
            self.model.actuator_biasprm[idx][2] = self.default_biasprm[idx][2]
            self.model.actuator_gainprm[idx][0] = self.default_gainprm[idx][0]
        elif mode == "torque":
            self.model.actuator_biasprm[idx][2] = 0
            self.model.actuator_gainprm[idx][0] = 0

    def get_sensor_feedback(
        self,
        robot: Literal["Kinova", "Dingo"],
        prop: Literal["position", "velocity", "torque"],
    ) -> list[float]:
        """Return position, velocity or torque feedback for kinova arm or dingo base."""
        match robot:
            case "Kinova":
                actuators = 6
            case "Dingo":
                actuators = 4

        feedback = []
        for n in range(actuators):
            idx = mj_name2id(self.model, mjtObj.mjOBJ_SENSOR, f"{robot}_{prop}_{n}")
            feedback.append(self.data.sensordata[idx])
        return feedback

    def set_ctrl_value(
        self,
        robot: Literal["Kinova", "Dingo"],
        prop: Literal["position", "velocity", "torque"],
        values: list[float],
    ) -> None:
        """Set position, velocity or torque command for kinova arm or dingo base."""
        for n, value in enumerate(values):
            idx = mj_name2id(self.model, mjtObj.mjOBJ_ACTUATOR, f"{robot}_{prop}_{n}")
            self.data.ctrl[idx] = value

    def ctrl_increment(
        self,
        robot: Literal["Kinova", "Dingo"],
        prop: Literal["position", "velocity", "torque"],
        joint: int,
        increment: float,
    ) -> None:
        """Control increment."""
        idx = mj_name2id(self.model, mjtObj.mjOBJ_ACTUATOR, f"{robot}_{prop}_{joint}")
        self.data.ctrl[idx] += increment
