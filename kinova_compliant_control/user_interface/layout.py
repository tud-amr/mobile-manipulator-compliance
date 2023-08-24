from typing import Literal
import dearpygui.dearpygui as dpg
from dearpygui_ext.logger import mvLogger
from kinova.kortex_client import KortexClient
from compliant_controller import controller, calibration
from .logger import Logger

PADDING = 65
LAST_ROW = 470
SPACER = 10


class Layout:
    """Defines the layout of the interface."""

    def __init__(
        self, name: str, width: int, height: int, client: KortexClient
    ) -> None:
        self.name = name
        self.width = width
        self.height = height
        self.client = client

    def create_ui(self) -> None:
        """Create the ui."""
        dpg.create_context()
        dpg.create_viewport(title=self.name)
        dpg.setup_dearpygui()

        self.create_theme()
        self.load_actuator_info()
        self.load_info()
        self.load_control()
        self.load_console()

        dpg.show_viewport()
        self.start_render_loop()
        dpg.destroy_context()

    def load_console(self) -> None:
        """Load a console."""
        x = 0
        y = 350
        w = self.width
        h = self.height - y - LAST_ROW - PADDING - SPACER
        tag = dpg.generate_uuid()
        dpg.add_window(
            label="Console",
            width=w,
            height=h,
            pos=[x, y],
            tag=tag,
            no_move=True,
            no_resize=True,
            no_collapse=True,
            no_close=True,
        )
        Logger.logger = mvLogger(parent=tag)

    def load_control(self) -> None:
        """Load control ui."""
        w = self.width / 2
        h = LAST_ROW
        x = w
        y = self.height - (h + PADDING)

        c = self.client
        HL_cal = calibration.HighLevelCalibration(self.client)
        LL_cal = calibration.LowLevelCalibration(self.client)

        grav = controller.CompensateGravityAndFriction(self.client)
        imp = controller.Impedance(self.client)
        cart_imp = controller.CartesianImpedance(self.client)
        with self.create_window("Control", w, h, [x, y]):
            dpg.add_text("Calibrate:")
            self.button("HL Calibration", HL_cal.calibrate_all_joints, c.HLC_available)
            self.button("LL Calibration", LL_cal.calibrate_all_joints, c.HLC_available)

            dpg.add_text("High Level:")
            self.button("Home", c.home, c.HLC_available)
            self.button("Zero", c.zero, c.HLC_available)
            self.button("Retract", c.retract, c.HLC_available)
            self.button("Start LLC", c.start_LLC, c.HLC_available)

            dpg.add_text("Low Level:")
            self.button("Stop LLC", c.stop_LLC, c.LLC_available)
            self.button("Gravity", grav.connect_to_LLC, c.LLC_available)
            self.button("Impedance", imp.connect_to_LLC, c.LLC_available)
            self.button("Cartesian Impedance", cart_imp.connect_to_LLC, c.LLC_available)
            self.button("Stop LLC task", c.disconnect_LLC, c.LLC_connected)

            dpg.add_text("Compensate:")
            self.checkbox(
                "Friction",
                controller.Controller.toggle_CF,
                controller.Controller.get_CF,
            )

            dpg.add_text("Active joints:")
            for n in range(self.client.actuator_count):
                self.checkbox(
                    f"Joint {n}",
                    lambda x, y, z: self.client.state.toggle_joint(int(z)),
                    self.client.state.get_joint_state,
                    [n],
                    str(n),
                )

    def load_info(self) -> None:
        """Load info ui."""
        w = self.width / 2
        h = LAST_ROW
        x = 0
        y = self.height - (h + PADDING)
        with self.create_window("Info", w, h, [x, y]):
            self.text("Update rate: ", self.client.get_update_rate)
            dpg.add_spacer()

            dpg.add_text("Actuator modes:")
            for n in range(self.client.actuator_count):
                self.text(f"Joint {n}: ", self.client.get_control_mode, [n])
            dpg.add_spacer()

            self.text("Servoing: ", self.client.get_servoing_mode)
            dpg.add_spacer()

            dpg.add_text("Current/Torque ratios:")
            for n in range(self.client.actuator_count):
                self.text(f"Joint {n}: ", self.client.state.get_ratio, [n])
            dpg.add_spacer()

            self.button("Clear faults", self.client.clear_faults)

    def load_actuator_info(self) -> None:
        """Load actuator info ui."""
        w2 = self.width / 2
        with self.create_window("Positions", w2, 0, [0, 0]):
            self.bars("position", w2)

        with self.create_window("Velocities", w2, 0, [w2, 0]):
            self.bars("velocity", w2)

        with self.create_window("Currents", w2, 0, [0, 170]):
            self.bars("current", w2)

        with self.create_window("Torques", w2, 0, [w2, 170]):
            self.bars("torque", w2)

    def checkbox(
        self,
        label: str,
        callback: callable,
        value_call: callable,
        args: list = None,
        callback_data: any = None,
    ) -> None:
        """Create a dearpygui checkbox."""
        tag = dpg.generate_uuid()
        dpg.add_checkbox(label=label, callback=callback, tag=tag)
        if callback_data:
            dpg.set_item_user_data(tag, callback_data)
        DynamicWidget("value", tag, value_call, args=args)

    def button(self, label: str, callback: callable, state: callable = None) -> None:
        """Create a dearpygui button."""
        tag = dpg.generate_uuid()
        dpg.add_button(label=label, callback=callback, tag=tag)
        if state:
            DynamicWidget("bool", tag, state)

    def text(self, label: str, call: callable, args: list = None) -> None:
        """Create a dearpygui text."""
        tag = dpg.generate_uuid()
        with dpg.group(horizontal=True):
            dpg.add_text(default_value=label)
            dpg.add_text(tag=tag)
        DynamicWidget("value", tag, call, args)

    def bars(self, prop: str, width: int) -> None:
        """Create dearpygui progress bars."""
        call: callable
        if prop == "position":
            call = self.client.get_position
        elif prop == "velocity":
            call = self.client.get_velocity
        elif prop == "current":
            call = self.client.get_current
        elif prop == "torque":
            call = self.client.get_torque
        for n in range(self.client.actuator_count):
            tag = dpg.generate_uuid()
            dpg.add_progress_bar(tag=tag, width=width)
            DynamicWidget("value", tag, call, [n, True])

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
        while dpg.is_dearpygui_running():
            DynamicWidget.update_all()
            dpg.render_dearpygui_frame()

    def create_window(
        self,
        label: str,
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
        call: callable,
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
