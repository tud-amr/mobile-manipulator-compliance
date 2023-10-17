import glfw
import dearpygui.dearpygui as dpg
from typing import Literal
from user_interface.window_commands import WindowCommands
from dataclasses import dataclass


@dataclass
class Joint:
    """Data of a joint."""

    name: str
    position: float = 0
    speed: float = 0
    current: float = 0
    voltage: float = 0


class Controller:
    """Used to tune the PID controller of the Dingo."""

    def __init__(self, callback: callable = lambda: None) -> None:
        self.joints = [Joint(f"joint{n}") for n in range(6)]
        self.mode = "HLC"
        self.data_names = ["position", "speed", "current", "voltage"]
        self.start_position = None
        self.callback = callback

        self.define_ui_parameters()
        self.create_ui()
        self.window_commands.add_window(self.window_name)
        self.window_commands.add_command(["replace", (self.window_name, *self.pose)])
        self.window_commands.start_in_new_thread()

    def HLC(self) -> bool:
        """Returns whether the current mode is HLC."""
        return self.mode == "HLC"

    def LLC(self) -> bool:
        """Returns whether the current mode is LLC."""
        return self.mode == "LLC"

    def LLC_task(self) -> bool:
        """Returns whether a LLC task is active."""
        return self.mode == "LLC_task"

    def define_ui_parameters(self) -> None:
        """Define the UI parameters."""
        glfw.init()
        self.window_commands = WindowCommands(1)
        self.window_name = "Kinova-Controller"
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
        dpg.create_viewport(title=self.window_name)
        dpg.setup_dearpygui()

        self.create_theme()
        self.create_plot("position", [0, 0])
        self.create_plot("speed", [self.w_plt, 0])
        self.create_plot("voltage", [0, self.h_plt])
        self.create_plot("current", [self.w_plt, self.h_plt])
        self.load_control([0, 2 * self.h_plt])

        dpg.show_viewport()

    def create_plot(self, data_name: str, pos: list) -> None:
        """Create a plot."""
        bar_center = 0.5
        with self.window(None, self.w_plt, self.h_plt, pos), dpg.plot(
            height=-1, width=-1
        ):
            dpg.add_plot_axis(
                dpg.mvXAxis,
                label=data_name,
                tag=f"{data_name}_x_axis",
                no_gridlines=True,
                no_tick_labels=True,
                no_tick_marks=True,
            )
            dpg.add_plot_axis(dpg.mvYAxis, tag=f"{data_name}_y_axis")
            for joint in self.joints:
                dpg.add_bar_series(
                    [bar_center],
                    [getattr(joint, data_name)],
                    weight=0.8,
                    parent=f"{data_name}_y_axis",
                    tag=f"{joint.name}_{data_name}",
                )
                bar_center += 1
            dpg.set_axis_limits(f"{data_name}_x_axis", 0, len(self.joints))
            dpg.set_axis_limits(f"{data_name}_y_axis", -5, 5)
            dpg.add_plot_axis(
                dpg.mvYAxis,
                label=" ",
                no_gridlines=True,
                no_tick_labels=True,
                no_tick_marks=True,
            )

    def load_control(self, pos: list) -> None:
        """Load control ui."""
        w = self.w_plt
        h = self.h_plt

        with self.window("Control", w, h, pos):
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

    def update_feedback(self) -> None:
        """Update the feedback."""
        for joint in self.joints:
            for data_name in self.data_names:
                data = dpg.get_value(f"{joint.name}_{data_name}")
                data[1] = [getattr(joint, data_name)]
                dpg.set_value(f"{joint.name}_{data_name}", data)

    def button(self, label: str, state: callable = None) -> None:
        """Create a dearpygui button."""
        dpg.add_button(label=label, callback=self.callback, tag=label)
        if state:
            DynamicWidget("bool", label, state)

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
            width=width,
            height=height,
            pos=pos,
            tag=tag,
        )

    def get_label_from_tag(self, tag: int) -> None:
        """Get the item label from tag."""
        return dpg.get_item_label(tag)

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
            DynamicWidget.update_all()
            dpg.render_dearpygui_frame()


class DynamicWidget:
    """Contains the dynamic widgets that need to be updated at render."""

    widgets: list["DynamicWidget"] = []

    @staticmethod
    def update_all() -> None:
        """Update all dynamic widgets."""
        for widget in DynamicWidget.widgets:
            widget.update()

    def __init__(
        self,
        update_type: Literal["value", "bool"],
        tag: str,
        call: callable = lambda: None,
        args: list = None,
    ) -> None:
        self.update_type = update_type
        self.tag = tag
        self.call = call
        self.args = args if args else []
        DynamicWidget.widgets.append(self)

    @property
    def value(self) -> any:
        """Get the dynamic value."""
        return self.call(*self.args)

    def update(self) -> None:
        """Update dynamic widget."""
        if self.update_type == "value":
            dpg.set_value(self.tag, self.value)
        elif self.update_type == "bool":
            dpg.configure_item(self.tag, enabled=self.value)
