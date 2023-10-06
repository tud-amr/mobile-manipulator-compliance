import dearpygui.dearpygui as dpg
from collections import deque


class Controller:
    """Used to tune the PID controller of the Dingo."""

    def __init__(self) -> None:
        self.actuator_name = "rear_right_wheel"
        self.actuator_id = 5
        self.target = 0
        self.start_position = None

        self.define_ui_parameters()
        self.create_ui()

    def define_ui_parameters(self) -> None:
        """Define the UI parameters."""
        self.width = 1200
        self.height = 600
        self.bar_height = 30
        self.data_x = deque()
        self.position = deque()
        self.targets = deque()
        self.voltage = deque()
        self.current = deque()
        self.n_samples = 1000
        self.actuators_ready = False

    def create_ui(self) -> None:
        """Create the ui."""
        self.active = True
        dpg.create_context()
        dpg.create_viewport(
            title="PID Tuner",
            min_width=self.width,
            max_width=self.width,
            min_height=self.height,
            max_height=self.height,
        )
        dpg.setup_dearpygui()
        dpg.show_viewport()

        self.create_plot()

    def create_plot(self) -> None:
        """Create a plot."""
        with dpg.window(
            width=self.width,
            height=self.height - self.bar_height,
            pos=[0, self.bar_height],
            no_resize=True,
            no_move=True,
            no_close=True,
            no_collapse=True,
            no_title_bar=True,
            no_bring_to_front_on_focus=True,
        ):
            with dpg.plot(height=-1, width=-1):
                dpg.add_plot_legend()
                dpg.add_plot_axis(
                    dpg.mvXAxis,
                    tag="x_axis",
                    no_tick_marks=True,
                    no_gridlines=True,
                    no_tick_labels=True,
                )
                dpg.add_plot_axis(
                    dpg.mvYAxis, label="Amps", tag="y_axis", no_gridlines=True
                )
                dpg.set_axis_limits("y_axis", -5, 5)
                dpg.add_line_series(
                    x=list(self.data_x),
                    y=list(self.position),
                    label="Position",
                    parent="y_axis",
                    tag="plot_position",
                )
                dpg.add_line_series(
                    x=list(self.voltage),
                    y=list(self.current),
                    label="Voltage",
                    parent="y_axis",
                    tag="plot_voltage",
                )
                dpg.add_line_series(
                    x=list(self.data_x),
                    y=list(self.current),
                    label="Current",
                    parent="y_axis",
                    tag="plot_current",
                )
                dpg.add_line_series(
                    x=list(self.data_x),
                    y=list(self.targets),
                    label="Target",
                    parent="y_axis",
                    tag="plot_target",
                )

    def update_data(self, x, position, voltage, current) -> None:
        """Update the data."""
        if self.start_position is None:
            self.start_position = position

        # error = position - self.start_position
        # self.target = -0.1 * error
        self.target = 1

        self.data_x.append(x)
        self.position.append(position)
        self.voltage.append(voltage)
        print(voltage)
        self.current.append(current)
        self.targets.append(self.target)
        while len(self.data_x) > self.n_samples:
            self.data_x.popleft()
            self.position.popleft()
            self.voltage.popleft()
            self.current.popleft()
            self.targets.popleft()

        dpg.set_value(
            "plot_position",
            [list(self.data_x), list(self.position)],
        )
        dpg.set_value(
            "plot_voltage",
            [list(self.data_x), list(self.voltage)],
        )
        dpg.set_value(
            "plot_current",
            [list(self.data_x), list(self.current)],
        )
        dpg.set_value(
            "plot_target",
            [list(self.data_x), list(self.targets)],
        )
        dpg.fit_axis_data("x_axis")

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
