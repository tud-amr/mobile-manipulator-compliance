from typing import Literal
from threading import Thread
import numpy as np
import importlib.resources as pkg_resources
import mujoco
import mujoco.viewer
import kinova.models as models
import time
import os
import re

VISUALIZATION_SYNC_RATE = 60  # Hz
MODEL = "GEN3-LITE.xml"


class MujocoViewer:
    """Provides the mujoco visualization of the robot."""

    def __init__(self) -> None:
        xml = str(pkg_resources.files(models) / MODEL)
        self.model = mujoco.MjModel.from_xml_path(xml)
        self.data = mujoco.MjData(self.model)
        self.name = re.search("b'(.*?)\\\\", str(self.model.names))[1]
        self.callback = lambda: None
        self.active = True
        move_target_thread = Thread(target=self.move_target_loop)
        move_target_thread.start()

    @property
    def target(self) -> np.ndarray:
        """Get the position of the target mocap body."""
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "target") - 1
        return self.data.mocap_pos[body_id]

    def update_marker(
        self, marker: Literal["end_effector", "target"], pos: np.ndarray
    ) -> None:
        """Update the given marker."""
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, marker) - 1
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
            self.update_marker("target", new_target_pos)
            time.sleep(1 / frequency)

    def toggle_automove(self) -> None:
        self.automove_target = not self.automove_target

    def key_callback(self, key: int) -> None:
        """Key callback."""
        if key == 256:
            os.system(f"wmctrl -c {self.name}")
            self.active = False

    def start_simulation(self) -> None:
        """Start a mujoco simulation without rendering."""
        while True:
            step_start = time.time()
            mujoco.mj_step(self.model, self.data)
            time_until_next_step = self.model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    def start_visualization(self) -> None:
        """Start the mujoco visualization."""
        with mujoco.viewer.launch_passive(
            self.model, self.data, key_callback=self.key_callback
        ) as self.viewer:
            sync_time = time.time()
            while self.active:
                step_start = time.time()
                mujoco.mj_forward(self.model, self.data)
                if time.time() > sync_time:
                    self.viewer.sync()
                    self.callback()
                    sync_time += 1 / VISUALIZATION_SYNC_RATE
                time_until_next_step = self.model.opt.timestep - (
                    time.time() - step_start
                )
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
