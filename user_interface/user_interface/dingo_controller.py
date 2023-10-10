import dearpygui.dearpygui as dpg
from dataclasses import dataclass


@dataclass
class Wheel:
    name: str
    start_position: float = None
    position: float = 0
    speed: float = 0
    voltage: float = 0
    current: float = 0


class Controller:
    """Used to tune the PID controller of the Dingo."""

    def __init__(self) -> None:
        self.wheels = [
            Wheel("front_left_wheel"),
            Wheel("front_right_wheel"),
            Wheel("rear_left_wheel"),
            Wheel("rear_right_wheel"),
        ]
        self.data_names = ["position", "speed", "voltage", "current"]
        self.start_position = None

        self.define_ui_parameters()
        self.create_ui()

    def define_ui_parameters(self) -> None:
        """Define the UI parameters."""
        self.width = 400
        self.height = 600
        self.bar_height = 30

    def create_ui(self) -> None:
        """Create the ui."""
        self.active = True
        dpg.create_context()
        dpg.create_viewport(
            title="Dingo Controller",
            min_width=self.width,
            max_width=self.width,
            min_height=self.height,
            max_height=self.height,
        )
        dpg.setup_dearpygui()
        dpg.show_viewport()

        self.create_plot("position", [0, 0])
        self.create_plot("speed", [int(self.width / 2), 0])
        self.create_plot("voltage", [0, self.height / 2])
        self.create_plot("current", [int(self.width / 2), self.height / 2])

    def create_plot(self, data_name: str, pos: list) -> None:
        """Create a plot."""
        with dpg.window(
            width=int(self.width / 2),
            height=int(self.height / 2),
            pos=pos,
            no_resize=True,
            no_move=True,
            no_close=True,
            no_collapse=True,
            no_title_bar=True,
            no_bring_to_front_on_focus=True,
        ):
            with dpg.plot(height=-1, width=-1):
                dpg.add_plot_axis(
                    dpg.mvXAxis,
                    label=data_name,
                    tag=f"{data_name}_x_axis",
                    no_gridlines=True,
                    no_tick_labels=True,
                    no_tick_marks=True,
                )
                dpg.add_plot_axis(dpg.mvYAxis, tag=f"{data_name}_y_axis")
                bar_center = 0.5
                for wheel in self.wheels:
                    dpg.add_bar_series(
                        [bar_center],
                        [getattr(wheel, data_name)],
                        weight=0.9,
                        parent=f"{data_name}_y_axis",
                        tag=f"{wheel.name}_{data_name}",
                    )
                    bar_center += 1
                dpg.set_axis_limits(f"{data_name}_x_axis", 0, 4)
                dpg.set_axis_limits(f"{data_name}_y_axis", -5, 5)
                dpg.add_plot_axis(
                    dpg.mvYAxis,
                    label=" ",
                    no_gridlines=True,
                    no_tick_labels=True,
                    no_tick_marks=True,
                )

    def update_data(self) -> None:
        """Update the data."""
        for wheel in self.wheels:
            for data_name in self.data_names:
                data = dpg.get_value(f"{wheel.name}_{data_name}")
                data[1] = [getattr(wheel, data_name)]
                dpg.set_value(f"{wheel.name}_{data_name}", data)

    def close(self) -> None:
        """Callback for keyboard input."""
        print("Close")
        self.active = False

    def start_render_loop(self) -> None:
        """Start the render loop."""
        with dpg.handler_registry():
            dpg.add_key_press_handler(dpg.mvKey_Escape, callback=self.close)
        while self.active:
            dpg.render_dearpygui_frame()
