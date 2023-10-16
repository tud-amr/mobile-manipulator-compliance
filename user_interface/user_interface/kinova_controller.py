import dearpygui.dearpygui as dpg
from dataclasses import dataclass


@dataclass
class Joint:
    name: str
    position: float = 0
    speed: float = 0
    current: float = 0
    voltage: float = 0


class Controller:
    """Used to tune the PID controller of the Dingo."""

    def __init__(self) -> None:
        self.joints = [Joint(f"joint{n}") for n in range(6)]
        self.data_names = ["position", "speed", "current", "voltage"]
        self.start_position = None

        self.define_ui_parameters()
        self.create_ui()

    def define_ui_parameters(self) -> None:
        """Define the UI parameters."""
        self.w_win = 400
        self.h_win = 900
        self.w_plt = int(self.w_win / 2)
        self.h_plt = int(self.h_win / 3)

    def create_ui(self) -> None:
        """Create the ui."""
        self.active = True
        dpg.create_context()
        dpg.create_viewport(
            title="Kinova Controller",
            min_width=self.w_win,
            max_width=self.w_win,
            min_height=self.h_win,
            max_height=self.h_win,
        )
        dpg.setup_dearpygui()
        dpg.show_viewport()

        self.create_plot("position", [0, 0])
        self.create_plot("speed", [self.w_plt, 0])
        self.create_plot("voltage", [0, self.h_plt])
        self.create_plot("current", [self.w_plt, self.h_plt])

    def create_plot(self, data_name: str, pos: list) -> None:
        """Create a plot."""
        with dpg.window(
            width=self.w_plt,
            height=self.h_plt,
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

    def update_feedback(self) -> None:
        """Update the feedback."""
        for joint in self.joints:
            for data_name in self.data_names:
                data = dpg.get_value(f"{joint.name}_{data_name}")
                data[1] = [getattr(joint, data_name)]
                dpg.set_value(f"{joint.name}_{data_name}", data)

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
