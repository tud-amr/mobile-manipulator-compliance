import glfw
import dearpygui.dearpygui as dpg
from threading import Thread
from dataclasses import dataclass
import time


@dataclass
class Joint:
    """Data of a joint."""

    index: int

    # continuous feedback:
    position: float = 0
    speed: float = 0
    current: float = 0

    # state:
    active: bool = True
    mode: str = "?"
    ratio: float = 0
    fric_d: float = 0
    fric_s: float = 0

    @property
    def name(self) -> str:
        """Return the name of the joint."""
        return f"joint{self.index}"

    @property
    def feedbacks(self) -> list[str]:
        """Return the feedbacks types."""
        return ["position", "speed", "current"]


@dataclass
class Wheel:
    """Data of a wheel."""

    name: str

    # continuous feedback:
    encoder_position: float = 0
    zero_position: float = 0
    speed: float = 0
    power: float = 0

    @property
    def position(self) -> float:
        """Return the current position."""
        return self.encoder_position - self.zero_position

    @property
    def feedbacks(self) -> list[str]:
        """Return the feedback types."""
        return ["position", "speed", "power"]


class UserInterface:
    """Used to tune the PID controller of the Dingo."""

    def __init__(self, callback: callable = lambda: None) -> None:
        self.joints = [Joint(n) for n in range(6)]
        self.wheels = [
            Wheel(f"{fr}_{lr}_wheel")
            for fr in ["front", "rear"]
            for lr in ["left", "right"]
        ]
        self.mode = "waiting"
        self.update_rate = 0
        self.servoing = "?"
        self.compensate_friction = False
        self.automove_target = False
        self.toggle_automove_target = lambda: None
        self.callback = callback

        self.define_ui_parameters()
        self.create_ui()

    @property
    def HLC(self) -> bool:
        """Returns whether the current mode is HLC."""
        return self.mode == "HLC"

    @property
    def LLC(self) -> bool:
        """Returns whether the current mode is LLC."""
        return self.mode == "LLC"

    @property
    def LLC_task(self) -> bool:
        """Returns whether a LLC task is active."""
        return self.mode == "LLC_task"

    def define_ui_parameters(self) -> None:
        """Define the UI parameters."""
        glfw.init()
        self.window_name = "Compliant Mobile Manipulation - User Interface"
        w_screen, h_screen = glfw.get_video_mode(glfw.get_primary_monitor()).size
        self.w_win = int(w_screen / 3)
        self.h_win = h_screen
        self.w_plt = int(self.w_win / 2)
        self.h_plt = int(self.h_win / 3)
        self.pose = [0, 0, self.w_win, self.h_win]

    def create_ui(self) -> None:
        """Create the ui."""
        self.active = True
        dpg.create_context()
        dpg.create_viewport(
            title=self.window_name,
            width=self.w_win,
            height=self.h_win,
            x_pos=0,
            y_pos=0,
            resizable=False,
            decorated=False,
        )
        dpg.setup_dearpygui()
        self.create_theme()

        w2 = int(self.w_win / 2)
        w3 = int(self.w_win / 3)
        h3 = int(self.h_win / 3)
        h6 = int(self.h_win / 6)

        self.create_plot("joint_position", w3, h3, [0, 0])
        self.create_plot("joint_speed", w3, h3, [w3, 0])
        self.create_plot("joint_current", w3, h3, [2 * w3, 0])

        self.load_control(w2, h3, [0, h3])
        self.load_state(w2, h6, [w2, h3])
        self.load_info(w2, h6, [w2, h3 + h6])

        self.create_plot("wheel_position", w3, h3, [0, 2 * h3])
        self.create_plot("wheel_speed", w3, h3, [w3, 2 * h3])
        self.create_plot("wheel_power", w3, h3, [2 * w3, 2 * h3])

        dpg.show_viewport()

    def create_plot(self, name: str, width: int, height: int, pos: list) -> None:
        """Create a plot."""
        limits: dict
        bar_center = 0.5
        label = name.split("_")[1]
        with self.window(None, width, height, pos), dpg.plot(height=-1, width=-1):
            dpg.add_plot_axis(
                dpg.mvXAxis,
                label=label,
                tag=f"{name}_x_axis",
                no_gridlines=True,
                no_tick_labels=True,
                no_tick_marks=True,
            )
            dpg.add_plot_axis(dpg.mvYAxis, tag=f"{name}_y_axis")
            if "joint" in name:
                limits = {"position": 2.6, "speed": 4, "current": 15}
                for joint in self.joints:
                    dpg.add_bar_series(
                        [bar_center],
                        [getattr(joint, label)],
                        weight=0.8,
                        parent=f"{name}_y_axis",
                        tag=f"{joint.name}_{label}",
                    )
                    bar_center += 1
            elif "wheel" in name:
                limits = {"position": 10, "speed": 5, "power": 10}
                for wheel in self.wheels:
                    dpg.add_bar_series(
                        [bar_center],
                        [getattr(wheel, label)],
                        weight=0.8,
                        parent=f"{name}_y_axis",
                        tag=f"{wheel.name}_{label}",
                    )
                    bar_center += 1
            dpg.set_axis_limits(f"{name}_x_axis", 0, len(self.joints))
            dpg.set_axis_limits(f"{name}_y_axis", -limits[label], limits[label])
            dpg.add_plot_axis(
                dpg.mvYAxis,
                label=" ",
                no_gridlines=True,
                no_tick_labels=True,
                no_tick_marks=True,
            )

    def load_info(self, width: int, height: int, pos: list) -> None:
        """Load info."""
        with self.window("Info", width, height, pos, tag="window_info"):
            pass
        thread = Thread(target=self.update_info)
        thread.start()

    def update_info(self) -> None:
        """Update info."""
        while self.active:
            if dpg.does_item_exist(item="group_info"):
                dpg.delete_item(item="group_info")
            with dpg.group(tag="group_info", parent="window_info"):
                dpg.add_text(f"Update rate: {self.update_rate}")
                dpg.add_text(f"Servoing: {self.servoing}")
            time.sleep(0.5)

    def load_control(self, width: int, height: int, pos: list) -> None:
        """Load the control window."""
        with self.window("Control", width, height, pos, tag="window_control"):
            pass
        self.update_control()

    def update_control(self) -> None:
        """Update the control window."""
        if dpg.does_item_exist(item="group_control"):
            dpg.delete_item(item="group_control")
        with dpg.group(parent="window_control", tag="group_control"):
            dpg.add_text("High Level:")
            with dpg.group(horizontal=True):
                self.button("Home", self.HLC)
                self.button("Zero", self.HLC)
                self.button("Retract", self.HLC)
            dpg.add_text("Switch:")
            with dpg.group(horizontal=True):
                self.button("Start LLC", self.HLC)
                self.button("Stop LLC", self.LLC)
                self.button("Stop LLC Task", self.LLC_task)
            dpg.add_text("Low Level:")
            with dpg.group(horizontal=True):
                self.button("Gravity", self.LLC)
                self.button("Impedance", self.LLC)
                self.button("Cartesian Impedance", self.LLC)
            dpg.add_text("Calibration:")
            with dpg.group(horizontal=True):
                self.button("HL Calibration", self.HLC)
                self.button("LL Calibration", self.HLC)
            dpg.add_spacer(height=10)
            dpg.add_text("Settings:")
            with dpg.group():
                self.checkbox("Compensate friction", enabled=self.compensate_friction)
                self.checkbox(
                    "Automove target",
                    enabled=self.automove_target,
                    callback=self.toggle_automove_target,
                )
                self.button("Clear Faults", True)
                self.button("Reset wheels", True, self.reset_wheel_positions)

    def reset_wheel_positions(self) -> None:
        """Reset the wheel positions."""
        for wheel in self.wheels:
            wheel.zero_position = wheel.encoder_position

    def load_state(self, width: int, height: int, pos: list) -> None:
        """Load joint info window."""
        with self.window("State", width, height, pos, tag="window_state"):
            pass
        self.update_state()

    def update_state(self) -> None:
        """Update joint info."""
        if dpg.does_item_exist(item="table"):
            dpg.delete_item(item="table")
        with dpg.table(
            header_row=True,
            borders_outerH=True,
            borders_outerV=True,
            tag="table",
            parent="window_state",
        ):
            headers = ["#", "Mode:", "Ratio:", "fric_D:", "fric_S:"]
            for header in headers:
                dpg.add_table_column(label=header)
            for joint in self.joints:
                with dpg.table_row():
                    with dpg.group(horizontal=True):
                        self.checkbox(None, f"Toggle_{joint.index}", joint.active)
                        dpg.add_text(joint.index)
                    dpg.add_text(joint.mode)
                    dpg.add_text(joint.ratio)
                    dpg.add_text(round(joint.fric_d, 3))
                    dpg.add_text(round(joint.fric_s, 3))

    def update_kinova_plots(self) -> None:
        """Update the Kinova plots."""
        for joint in self.joints:
            for feedback in joint.feedbacks:
                data = dpg.get_value(f"{joint.name}_{feedback}")
                data[1] = [getattr(joint, feedback)]
                dpg.set_value(f"{joint.name}_{feedback}", data)

    def update_dingo_plots(self) -> None:
        """Update the Dingo plots."""
        for wheel in self.wheels:
            for feedback in wheel.feedbacks:
                data = dpg.get_value(f"{wheel.name}_{feedback}")
                data[1] = [getattr(wheel, feedback)]
                dpg.set_value(f"{wheel.name}_{feedback}", data)

    def button(
        self, label: str, enabled: bool = False, callback: callable = None
    ) -> None:
        """Create a dearpygui button."""
        if callback is None:
            callback = self.callback
        dpg.add_button(label=label, enabled=enabled, callback=callback, tag=label)

    def checkbox(
        self,
        label: str,
        tag: str = None,
        enabled: bool = False,
        callback: callable = None,
    ) -> None:
        """Create a dearpygui checkbox."""
        if tag is None:
            tag = label
        if callback is None:
            callback = self.callback
        dpg.add_checkbox(label=label, tag=tag, default_value=enabled, callback=callback)

    def window(
        self,
        label: str = None,
        width: int = 0,
        height: int = 0,
        pos: list = None,
        tag: int = None,
    ) -> dpg.contextmanager:
        """Create a dearpygui window."""
        if not tag:
            tag = dpg.generate_uuid()
        return dpg.window(
            label=label,
            no_move=True,
            no_resize=True,
            no_collapse=True,
            no_close=True,
            no_title_bar=True,
            width=width,
            height=height,
            min_size=[width, height],
            max_size=[width, height],
            pos=pos,
            tag=tag,
        )

    def close(self) -> None:
        """Callback for keyboard input."""
        print("Close")
        self.active = False

    def create_theme(self) -> None:
        """Create the theme."""
        # Colors from https://github.com/hoffstadt/DearPyGui_Ext/blob/master/dearpygui_ext/themes.py
        disabled = (0.50 * 255, 0.50 * 255, 0.50 * 255, 1.00 * 255)
        with dpg.theme() as t, dpg.theme_component(dpg.mvButton, enabled_state=False):
            dpg.add_theme_color(dpg.mvThemeCol_Button, disabled)
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, disabled)
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, disabled)
        dpg.bind_theme(t)

    def start_render_loop(self) -> None:
        """Start the render loop."""
        with dpg.handler_registry():
            dpg.add_key_press_handler(dpg.mvKey_Escape, callback=self.close)
        while self.active:
            dpg.render_dearpygui_frame()
