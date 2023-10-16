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
        self.active = True

    def key_callback(self, key: int) -> None:
        """Key callback."""
        if key == 256:
            os.system(f"wmctrl -c {self.name}")
            self.active = False

    def start_simulation(self) -> None:
        """Start the mujoco simulation."""
        self.viewer = mujoco.viewer._launch_internal(
            self.model,
            self.data,
            run_physics_thread=True,
            key_callback=self.key_callback,
        )

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
                    sync_time += 1 / VISUALIZATION_SYNC_RATE
                time_until_next_step = self.model.opt.timestep - (
                    time.time() - step_start
                )
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
