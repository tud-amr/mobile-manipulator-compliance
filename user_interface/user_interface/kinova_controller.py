import glfw
import dearpygui.dearpygui as dpg
from threading import Thread
from dataclasses import dataclass
import time


@dataclass
class Joint:
    """Data of a joint."""

    index: int

    @property
    def name(self) -> str:
        return f"joint{self.index}"

    # continious feedback:
    position: float = 0
    speed: float = 0
    current: float = 0
    voltage: float = 0

    # state:
    active: bool = True
    mode: str = "?"
    ratio: float = 0
    fric_d: float = 0
    fric_s: float = 0


class Controller:
    """Used to tune the PID controller of the Dingo."""

    def __init__(self, callback: callable = lambda: None) -> None:
        self.joints = [Joint(n) for n in range(6)]
        self.mode = "waiting"
        self.update_rate = 0
        self.servoing = "?"
        self.data_names = ["position", "speed", "current", "voltage"]
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
        dpg.create_viewport(
            title=self.window_name,
            width=self.w_win,
            height=self.h_win,
            x_pos=0,
            y_pos=0,
            resizable=False,
        )
        dpg.setup_dearpygui()

        self.create_theme()
        self.create_plot("position", [0, 0])
        self.create_plot("speed", [self.w_plt, 0])
        self.create_plot("voltage", [0, self.h_plt])
        self.create_plot("current", [self.w_plt, self.h_plt])
        self.load_control([0, 2 * self.h_plt])
        self.load_info([0, int(2.5 * self.h_plt)])
        self.load_state([self.w_plt, 2 * self.h_plt])

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

    def load_info(self, pos: list) -> None:
        """Load info."""
        w = self.w_plt
        h = int(self.h_plt / 2)
        with self.window("Info", w, h, pos, tag="window_info"):
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

    def load_control(self, pos: list) -> None:
        """Load the control window."""
        w = self.w_plt
        h = int(self.h_plt / 2)

        with self.window("Control", w, h, pos, tag="window_control"):
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

    def load_state(self, pos: list) -> None:
        """Load joint info window."""
        w = self.w_plt
        h = self.h_plt

        with self.window("State", w, h, pos, tag="window_state"):
            pass
        self.update_state()

    def update_state(self):
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
            headers = ["#", "Mode:", "Ratio:", "Dfric:", "Sfric:"]
            for header in headers:
                dpg.add_table_column(label=header)
            for joint in self.joints:
                with dpg.table_row():
                    with dpg.group(horizontal=True):
                        dpg.add_checkbox(
                            default_value=joint.active,
                            callback=self.callback,
                            tag=f"Toggle_{joint.index}",
                        )
                        dpg.add_text(joint.index)
                    dpg.add_text(joint.mode)
                    dpg.add_text(joint.ratio)
                    dpg.add_text(round(joint.fric_d, 3))
                    dpg.add_text(round(joint.fric_s, 3))

    def update_feedback(self) -> None:
        """Update the feedback."""
        for joint in self.joints:
            for data_name in self.data_names:
                data = dpg.get_value(f"{joint.name}_{data_name}")
                data[1] = [getattr(joint, data_name)]
                dpg.set_value(f"{joint.name}_{data_name}", data)

    def button(
        self, label: str, enabled: bool = False, callback: callable = None
    ) -> None:
        """Create a dearpygui button."""
        if callback is None:
            callback = self.callback
        dpg.add_button(label=label, enabled=enabled, callback=callback, tag=label)

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
