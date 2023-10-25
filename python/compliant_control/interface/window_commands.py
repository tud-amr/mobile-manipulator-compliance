import os
from threading import Thread
import time


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
        thread = Thread(target=self.start, daemon=True)
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
            time.sleep(0.1)
        while not ready:
            ready = True
            for window in self.windows:
                ready &= os.popen(f"xdotool search --name {window}").read() != ""
