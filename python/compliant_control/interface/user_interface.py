import glfw
from typing import Literal
import dearpygui.dearpygui as dpg
from threading import Thread
from dataclasses import dataclass
import time
from compliant_control.interface.joystick import Joystick
from compliant_control.interface.templates import (
    window,
    button,
    checkbox,
    create_plot,
    update_plot,
)


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

    def __init__(self) -> None:
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

        self.cb_kin = None
        self.cb_sim = None
        self.cb_din = None

        self.define_ui_parameters()

    @property
    def joint_names(self) -> list:
        """Returns the list of joint names."""
        return [joint.name for joint in self.joints]

    @property
    def wheel_names(self) -> list:
        """Returns the list of wheel names."""
        return [wheel.name for wheel in self.wheels]

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

        create_plot("pos", w3, h3, [0, 0], self.joint_names, 2.6)
        create_plot("vel", w3, h3, [w3, 0], self.joint_names, 4)
        create_plot("tor", w3, h3, [2 * w3, 0], self.joint_names, 15)

        self.load_control(w2, int(h3 * 0.85), [0, h3])
        self.load_info(w2, int(h3 * 0.15), [0, int(h3 * 1.85)])
        self.load_state(w2, h6, [w2, h3])
        Joystick(w2, h6, [w2, h3 + h6], self.cb_din)

        create_plot("pos", w3, h3, [0, 2 * h3], self.wheel_names, 10)
        create_plot("vel", w3, h3, [w3, 2 * h3], self.wheel_names, 5)
        create_plot("tor", w3, h3, [2 * w3, 2 * h3], self.wheel_names, 10)

        dpg.show_viewport()

    def load_info(self, width: int, height: int, pos: list) -> None:
        """Load info."""
        with window(width, height, pos, tag="window_info"):
            pass
        thread = Thread(target=self.update_info)
        thread.start()

    def update_info(self) -> None:
        """Update info."""
        while self.active:
            if dpg.does_item_exist(item="group_info"):
                dpg.delete_item(item="group_info")
            with dpg.group(tag="group_info", parent="window_info", horizontal=True):
                dpg.add_text(f"Update rate: {self.update_rate}")
                dpg.add_text(f"Servoing: {self.servoing}")
            time.sleep(0.5)

    def load_control(self, width: int, height: int, pos: list) -> None:
        """Load the control window."""
        with window(width, height, pos, tag="window_control"):
            pass
        self.update_control()

    def update_control(self) -> None:
        """Update the control window."""
        if dpg.does_item_exist(item="group_control"):
            dpg.delete_item(item="group_control")
        with dpg.group(parent="window_control", tag="group_control"):
            dpg.add_text("High Level:")
            with dpg.group(horizontal=True):
                button("Home", self.HLC, self.cb_kin)
                button("Zero", self.HLC, self.cb_kin)
                button("Retract", self.HLC, self.cb_kin)
            dpg.add_text("Switch:")
            with dpg.group(horizontal=True):
                button("Start LLC", self.HLC, self.cb_kin)
                button("Stop LLC", self.LLC, self.cb_kin)
                button("Stop LLC Task", self.LLC_task, self.cb_kin)
            dpg.add_text("Low Level:")
            with dpg.group(horizontal=True):
                button("Gravity", self.LLC, self.cb_kin)
                button("Impedance", self.LLC, self.cb_kin)
                button("Cartesian Impedance", self.LLC, self.cb_kin)
            dpg.add_text("Calibration:")
            with dpg.group(horizontal=True):
                button("HL Calibration", self.HLC, self.cb_kin)
                button("LL Calibration", self.HLC, self.cb_kin)
            dpg.add_spacer(height=10)
            dpg.add_text("Settings:")
            with dpg.group():
                checkbox("Compensate friction", self.compensate_friction, self.cb_kin)
                checkbox("Automove target", self.automove_target, self.cb_sim)
            with dpg.group(horizontal=True):
                button("Clear Faults", True, self.cb_kin)
                button("Reset wheels", True, self.reset_wheel_positions)

    def reset_wheel_positions(self) -> None:
        """Reset the wheel positions."""
        for wheel in self.wheels:
            wheel.zero_position = wheel.encoder_position

    def load_state(self, width: int, height: int, pos: list) -> None:
        """Load joint info window."""
        with window(width, height, pos, tag="window_state"):
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
                        checkbox(f"Toggle_{joint.index}", joint.active, self.cb_kin)
                        dpg.add_text(joint.index)
                    dpg.add_text(joint.mode)
                    dpg.add_text(joint.ratio)
                    dpg.add_text(round(joint.fric_d, 3))
                    dpg.add_text(round(joint.fric_s, 3))

    def update_bars(self, robot: Literal["Kinova", "Dingo"]) -> None:
        """Update the bar plots."""
        match robot:
            case "Kinova":
                for joint in self.joints:
                    update_plot("pos", joint.name, joint.position)
                    update_plot("vel", joint.name, joint.speed)
                    update_plot("tor", joint.name, joint.current)
            case "Dingo":
                for wheel in self.wheels:
                    update_plot("pos", wheel.name, wheel.position)
                    update_plot("vel", wheel.name, wheel.speed)
                    update_plot("tor", wheel.name, wheel.power)

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
