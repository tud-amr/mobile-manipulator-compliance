import glfw
import time
from threading import Thread
from typing import Literal, Callable
import dearpygui.dearpygui as dpg
from compliant_control.interface.data_classes import Joint, Wheel, State, Rates
from compliant_control.interface.joystick import Joystick
from compliant_control.interface.templates import (
    window,
    create_plot,
    update_plot,
    Group,
    Widget,
    Row,
    Table,
    Button,
    Checkbox,
    Text,
)

JOINTS = 6
WHEELS = 4


class UserInterface:
    """Used to tune the PID controller of the Dingo."""

    def __init__(self, callback: Callable) -> None:
        self.joints = [Joint(n) for n in range(JOINTS)]
        self.wheels = [Wheel(n) for n in range(WHEELS)]
        self.state = State()
        self.rates = Rates()
        thread = Thread(target=self.update_rates_loop)
        thread.start()

        Widget.general_callback = callback
        self.define_ui_parameters()

    @property
    def joint_names(self) -> list:
        """Returns the list of joint names."""
        return [joint.name for joint in self.joints]

    @property
    def wheel_names(self) -> list:
        """Returns the list of wheel names."""
        return [wheel.name for wheel in self.wheels]

    def reset_wheels(self) -> None:
        """Reset the wheels."""
        for wheel in self.wheels:
            wheel.reset()

    def update_state(self, mode: str = None) -> None:
        """Update the state."""
        self.state.mode = mode if mode is not None else self.state.mode
        Group.update_all()

    def update_rates_loop(self) -> None:
        """Update the rates."""
        while True:
            Group.update_all()
            time.sleep(1)

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
        create_plot("eff", w3, h3, [2 * w3, 0], self.joint_names, 15)

        self.load_control(w2, int(h3 * 0.85), [0, h3])
        self.load_info(w2, int(h3 * 0.15), [0, int(h3 * 1.85)])
        self.load_state(w2, h6, [w2, h3])
        self.joystick = Joystick(w2, h6, [w2, h3 + h6], Widget.general_callback)

        create_plot("pos", w3, h3, [0, 2 * h3], self.wheel_names, 50)
        create_plot("vel", w3, h3, [w3, 2 * h3], self.wheel_names, 1)
        create_plot("eff", w3, h3, [2 * w3, 2 * h3], self.wheel_names, 50)

        dpg.show_viewport()

    def load_control(self, width: int, height: int, pos: list) -> None:
        """Load the control window."""
        with window(width, height, pos, tag="window_control"):
            dpg.add_text("High Level:")
            Row(
                [
                    Button("Home"),
                    Button("Zero"),
                    Button("Retract"),
                    Button("Pref"),
                ],
                enabled=self.state.HLC,
            )
            Row(
                [Button("Start LLC"), Button("Calibrate"), Button("Start HLT")],
                enabled=self.state.HLC,
            )

            Row(
                [Button("Stop HLT")],
                enabled=self.state.HLT,
            )

            dpg.add_text("Switch:")
            Row(
                [Button("Stop LLC"), Button("Start LLC Task")],
                self.state.LLC,
            )
            Row([Button("Stop LLC Task")], self.state.LLC_task)

            dpg.add_text("Low Level:")
            Table(
                None,
                [
                    [
                        Text("Comp:"),
                        Checkbox("grav", self.state.get_comp_grav),
                        Checkbox("fric", self.state.get_comp_fric),
                        Text(""),
                    ],
                    [
                        Text("Imp:"),
                        Checkbox("arm", self.state.get_imp_arm),
                        Checkbox("null", self.state.get_imp_null),
                        Checkbox("base", self.state.get_imp_base),
                    ],
                ],
                self.state.LLC_task,
            )

            Row(
                [
                    Checkbox("Automove", self.state.get_automove_target),
                    Button("Reset"),
                    Button("Clear"),
                    Button("Refresh"),
                ],
                True,
            )
            Row([Text("Gripper:"), Button("Open"), Button("Close")], self.state.HLC)

    def load_info(self, width: int, height: int, pos: list) -> None:
        """Load info."""
        with window(width, height, pos, tag="window_info"):
            headers = self.rates.names
            widgets = [[Text("", lambda _x=x: self.rates.value(_x)) for x in headers]]
            self.info = Table(headers, widgets)

    def load_state(self, width: int, height: int, pos: list) -> None:
        """Load joint info window."""
        headers = ["#", "Mode:", "Ratio:", "Friction:"]
        widgets = [
            [
                Checkbox(str(joint.index), joint.is_active),
                Text("", joint.get_mode),
                Text("", joint.get_ratio),
                Text("", joint.get_friction),
            ]
            for joint in self.joints
        ]
        with window(width, height, pos):
            Table(headers, widgets, self.state.HLC)

    def update_bars(self, robot: Literal["Kinova", "Dingo"]) -> None:
        """Update the bar plots."""
        if robot == "Kinova":
            elements = self.joints
        elif robot == "Dingo":
            elements = self.wheels
        for prop in ["pos", "vel", "eff"]:
            for element in elements:
                name = getattr(element, "name")
                value = getattr(element, prop)
                update_plot(prop, name, value)

    def stop(self, *args: any) -> None:
        """Stop the render loop."""
        self.active = False

    def create_theme(self) -> None:
        """Create the theme."""
        # Colors from https://github.com/hoffstadt/DearPyGui_Ext/blob/master/dearpygui_ext/themes.py
        disabled = (0.50 * 255, 0.50 * 255, 0.50 * 255, 1.00 * 255)
        with dpg.theme() as t:
            with dpg.theme_component(dpg.mvButton, enabled_state=False):
                dpg.add_theme_color(dpg.mvThemeCol_Button, disabled)
                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, disabled)
                dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, disabled)
            with dpg.theme_component(dpg.mvCheckbox, enabled_state=False):
                dpg.add_theme_color(dpg.mvThemeCol_CheckMark, disabled)
        dpg.bind_theme(t)

    def start(self) -> None:
        """Start the render loop."""
        with dpg.handler_registry():
            dpg.add_key_press_handler(dpg.mvKey_Escape, callback=self.stop)
        while self.active:
            dpg.render_dearpygui_frame()
