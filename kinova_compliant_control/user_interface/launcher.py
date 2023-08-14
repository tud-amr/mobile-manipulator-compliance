import os
import glfw
from threading import Thread
from .layout import Layout
from .mujoco_viewer import MujocoViewer
from kinova.kortex_client import KortexClient


class Launcher:
    """Launches the user interface and mujoco."""

    def __init__(self, client: KortexClient, mujoco_viewer: MujocoViewer) -> None:
        self.client = client
        self.mujoco_viewer = mujoco_viewer

        glfw.init()
        self.width, self.height = glfw.get_video_mode(glfw.get_primary_monitor()).size

        self.window_commands = WindowCommands(2)
        self.window_commands.start_in_new_thread()
        self.start_dearpygui_viewer()
        self.start_mujoco_viewer()

    def start_dearpygui_viewer(self) -> None:
        """Start the dearpygui ui."""
        name = "Control-UI"
        pose = [0, 0, int(self.width * (1 / 4)), self.height]
        self.window_commands.add_window(name)
        self.window_commands.add_command(["replace", (name, *pose)])
        layout = Layout(name, pose[2], pose[3], self.client)
        thread = Thread(target=layout.create_ui, daemon=True)
        thread.start()

    def start_mujoco_viewer(self) -> None:
        """Start the mujoco viewer."""
        name = self.mujoco_viewer.name
        pose = [int(self.width / 4), 0, int(self.width * (3 / 4)), self.height]
        self.window_commands.add_window(name)
        self.window_commands.add_command(["replace", (name, *pose)])
        self.window_commands.add_command(["key", (name, "Tab")])
        self.window_commands.add_command(["key", (name, "Shift+Tab")])
        self.window_commands.add_command(["key", (name, "F2")])

        if self.client.simulate:
            self.mujoco_viewer.start_simulation()
        else:
            self.mujoco_viewer.start_visualization()


class WindowCommands:
    """Used to execute linux commands to modify opened windows."""

    def __init__(self, n_windows: int) -> None:
        self.n_windows = n_windows
        self.windows = []
        self.commands = []

    def add_command(self, command: list) -> None:
        """Add a command."""
        self.commands.append(command)

    def add_window(self, window: str) -> None:
        """Add a window."""
        self.windows.append(window)

    def start_in_new_thread(self) -> None:
        """Start the window commander in a new thread."""
        thread = Thread(target=self.start)
        thread.start()

    def start(self) -> None:
        """Start the window commander."""
        self.wait_till_ready()
        self.execute_window_commands()

    def execute_window_commands(self) -> None:
        """Execute the commands."""
        for window_command in self.commands:
            command: str
            if window_command[0] == "replace":
                command = "wmctrl -r {} -e 0,{},{},{},{}".format(*window_command[1])
            elif window_command[0] == "key":
                command = "xdotool key --window $(xdotool search --name {}) {}".format(
                    *window_command[1],
                )
            os.system(command)

    def wait_till_ready(self) -> None:
        """Wait till the windows are ready."""
        ready = False
        while len(self.windows) != self.n_windows:
            pass
        while not ready:
            ready = True
            for window in self.windows:
                ready &= os.popen(f"xdotool search --name {window}").read() != ""
