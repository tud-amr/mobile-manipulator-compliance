import dearpygui.dearpygui as dpg
from collections import deque


class Controller:
    """Used to tune the PID controller of the Dingo."""

    def __init__(self) -> None:
        self.actuator_name = "rear_right_wheel"
        self.actuator_id = 5

        self.define_ui_parameters()
        self.create_ui()

    def define_ui_parameters(self) -> None:
        """Define the UI parameters."""
        self.width = 1200
        self.height = 600
        self.bar_height = 30
        self.data_x = deque()
        self.position = deque()
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
                dpg.set_axis_limits("y_axis", 0, 5)
                dpg.add_line_series(
                    x=list(self.data_x),
                    y=list(self.position),
                    label="Target",
                    parent="y_axis",
                    tag="control_target",
                )
                dpg.add_line_series(
                    x=list(self.data_x),
                    y=list(self.current),
                    label="Feedback",
                    parent="y_axis",
                    tag="control_feedback",
                )

    def update_data(self, x, position, current) -> None:
        """Update the data."""
        self.data_x.append(x)
        self.position.append(position)
        self.current.append(current)
        while len(self.data_x) > self.n_samples:
            self.data_x.popleft()
            self.position.popleft()
            self.current.popleft()

        dpg.set_value(
            "control_target",
            [list(self.data_x), list(self.position)],
        )
        dpg.set_value(
            "control_feedback",
            [list(self.data_x), list(self.current)],
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


controller = Controller()
controller.start_render_loop()
