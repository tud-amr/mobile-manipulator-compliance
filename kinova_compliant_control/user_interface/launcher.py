import re
import mujoco
import mujoco.viewer
import time
import os
import glfw
from threading import Thread
from .layout import Layout
from user_interface.mujoco_viewer import MujocoViewer
from kinova.kortex_client import KortexClient

VISUALIZATION_SYNC_RATE = 60  # Hz


class Launcher:
    """Launches the user interface and mujoco."""

    def __init__(self, client: KortexClient, mujoco_viewer: MujocoViewer) -> None:
        self.client = client
        self.model = mujoco_viewer.model
        self.data = mujoco_viewer.data
        self.active = True

        glfw.init()
        self.width, self.height = glfw.get_video_mode(glfw.get_primary_monitor()).size
        self.stop = False
        self.paused = False
        self.windows = []
        self.window_commands = []

        self.start_dearpygui_viewer()
        self.add_mujoco_window_commands()
        thread = Thread(target=self.execute_window_commands)
        thread.start()
        if self.client.simulate:
            self.start_mujoco_simulation()
        else:
            self.start_mujoco_visualization()

    def key_callback(self, key: int) -> None:
        """Key callback."""
        if key == 256:
            os.system(f"wmctrl -c {self.name}")
            self.active = False

    def start_dearpygui_viewer(self) -> None:
        """Start the dearpygui ui."""
        name = "Control-UI"
        pose = [0, 0, int(self.width * (1 / 4)), self.height]
        self.windows.append(name)
        self.window_commands.append(["replace", (name, *pose)])
        layout = Layout(name, pose[2], pose[3], self.client)
        thread = Thread(target=layout.create_ui, daemon=True)
        thread.start()

    def add_mujoco_window_commands(self) -> None:
        """Add commands for the mujoco window."""
        name = re.search("b'(.*?)\\\\", str(self.model.names))[1]
        self.name = name
        pose = [int(self.width / 4), 0, int(self.width * (3 / 4)), self.height]
        self.windows.append(name)
        self.window_commands.append(["replace", (name, *pose)])
        self.window_commands.append(["key", (name, "Tab")])
        self.window_commands.append(["key", (name, "Shift+Tab")])
        self.window_commands.append(["key", (name, "F2")])

    def start_mujoco_simulation(self) -> None:
        """Start the mujoco simulation."""
        self.viewer = mujoco.viewer._launch_internal(
            self.model,
            self.data,
            run_physics_thread=True,
            key_callback=self.key_callback,
        )

    def start_mujoco_visualization(self) -> None:
        """Start the mujoco visualization."""
        self.viewer: mujoco.viewer.Handle
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

    def execute_window_commands(self) -> None:
        """Execute linux commands to modify opened windows."""
        ready = False
        while not ready:
            ready = True
            for window in self.windows:
                ready &= os.popen(f"xdotool search --name {window}").read() != ""

        for window_command in self.window_commands:
            command: str
            if window_command[0] == "replace":
                command = "wmctrl -r {} -e 0,{},{},{},{}".format(*window_command[1])
            elif window_command[0] == "key":
                command = "xdotool key --window $(xdotool search --name {}) {}".format(
                    *window_command[1],
                )
            os.system(command)
