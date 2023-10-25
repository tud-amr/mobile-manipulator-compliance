from typing import Literal
import importlib.resources as pkg_resources
import mujoco.viewer
from mujoco import MjModel, MjData, mj_name2id, mjtObj
import compliant_control.simulation.models as models
import time
import re
from compliant_control.interface.window_commands import WindowCommands
import glfw
import numpy as np
from threading import Thread

SYNC_RATE = 60
MODEL = "arm_and_base.xml"


class Simulation:
    """Provides the mujoco visualization of the robot."""

    def __init__(self, callback: callable) -> None:
        self.callback = callback
        xml = str(pkg_resources.files(models) / MODEL)
        self.model = MjModel.from_xml_path(xml)
        self.data = MjData(self.model)
        self.name = re.search("b'(.*?)\\\\", str(self.model.names))[1]
        self.default_biasprm = self.model.actuator_biasprm.copy()
        self.default_gainprm = self.model.actuator_gainprm.copy()
        self.active = True
        move_target_thread = Thread(target=self.move_target_loop)
        move_target_thread.start()

    @property
    def target(self) -> np.ndarray:
        """Get the position of the target mocap body."""
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "target") - 1
        return self.data.mocap_pos[body_id]

    def update_target(self, pos: np.ndarray) -> None:
        """Update the given marker."""
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "target") - 1
        self.data.mocap_pos[body_id] = pos

    def move_target_loop(self) -> None:
        """A loop that automatically moves the target."""
        self.automove_target = False
        frequency = 100
        target_rate = 0.05  # m/s
        target_step = target_rate / frequency
        target_rot = 90
        target_rot_rate = 30  # deg / s
        target_rot_step = target_rot_rate / frequency
        while self.active:
            if not self.automove_target:
                time.sleep(0.5)
                continue
            target_rot += target_rot_step
            step_x = np.cos(np.deg2rad(target_rot)) * target_step
            step_y = np.sin(np.deg2rad(target_rot)) * target_step
            step_z = 0
            new_target_pos = self.target + np.array([step_x, step_y, step_z])
            self.update_target(new_target_pos)
            time.sleep(1 / frequency)

    def toggle_automove_target(self) -> None:
        """Toggle automove of target."""
        print("toggle")
        self.automove_target = not self.automove_target

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

    def start(self) -> None:
        """Start a mujoco simulation without rendering."""
        self.load_window_commands()
        viewer = mujoco.viewer.launch_passive(
            self.model, self.data, key_callback=self.key_callback
        )
        n = 0
        last = time.time()
        sync = time.time()
        while self.active:
            step_start = time.time()
            mujoco.mj_step(self.model, self.data)
            self.callback()
            if time.time() > sync + (1 / SYNC_RATE):
                viewer.sync()
                sync = time.time()
            if time.time() > last + 1:
                print(n)
                n = 0
                last = time.time()
            n += 1
            time_until_next_step = self.model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    def key_callback(self, key: int) -> None:
        """Key callback."""
        if key == 256:
            self.stop()

    def stop(self, *args: any) -> None:
        """Stop the simulation."""
        self.active = False

    def load_window_commands(self) -> None:
        """Load the window commands."""
        glfw.init()
        window_commands = WindowCommands(1)
        width, height = glfw.get_video_mode(glfw.get_primary_monitor()).size
        pose = [int(width / 3), 0, int(width * (2 / 3)), height]
        window_commands = WindowCommands(1)
        window_commands.add_window(self.name)
        window_commands.add_command(["replace", (self.name, *pose)])
        window_commands.add_command(["key", (self.name, "Tab")])
        window_commands.add_command(["key", (self.name, "Shift+Tab")])
        window_commands.start_in_new_thread()
