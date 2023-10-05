import dearpygui.dearpygui as dpg
from dataclasses import dataclass


@dataclass
class Input:
    value: float
    minimum: float
    maximum: float
    step: float


class Tuner:
    """Used to tune the PID controller of the Dingo."""

    def __init__(self, set_gain_callback: callable) -> None:
        self.set_gain_callback = set_gain_callback
        self.actuator_name = "rear_right_wheel"
        self.actuator_id = 5

        self.inputs = {
            "target": Input(0.6, 0, 2, 0.1),
            "P": Input(0.2, 0, 1, 0.1),
            "I": Input(4, 0, 10, 1),
            "D": Input(0, 0, 0, 1),
        }

        self.define_ui_parameters()
        self.create_ui()

    def define_ui_parameters(self) -> None:
        """Define the UI parameters."""
        self.width = 1200
        self.height = 600
        self.bar_height = 30
        self.data_x = []
        self.target_y = []
        self.feedback_y = []
        self.n_samples = 300
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

        self.create_buttons()
        self.create_plot()

    def create_buttons(self) -> None:
        """Create the buttons."""
        with dpg.window(
            min_size=[self.width, self.bar_height],
            max_size=[self.width, self.bar_height],
            pos=[0, 0],
            no_resize=True,
            no_move=True,
            no_close=True,
            no_collapse=True,
            no_title_bar=True,
            no_scrollbar=True,
            no_bring_to_front_on_focus=True,
        ):
            with dpg.group(horizontal=True):
                for name_, input_ in self.inputs.items():
                    dpg.add_text(name_)
                    dpg.add_input_double(
                        width=100,
                        default_value=input_.value,
                        min_value=input_.minimum,
                        max_value=input_.maximum,
                        step=input_.step,
                        min_clamped=True,
                        max_clamped=True,
                        callback=self.button_callback,
                        tag="value_" + name_,
                    )
                    if name_ == "target":
                        dpg.add_spacer(width=250)

    def button_callback(self) -> None:
        """Callback of the buttons."""
        for tag in self.inputs.keys():
            value = dpg.get_value("value_" + tag)
            if self.inputs[tag].value != value:
                self.inputs[tag].value = value
                if tag != "target":
                    print(f"Set gain {tag} to {value}...")
                    self.set_gain_callback(tag, value)

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
                dpg.set_axis_limits("y_axis", 0, 2)
                dpg.add_line_series(
                    x=self.data_x,
                    y=self.target_y,
                    label="Target",
                    parent="y_axis",
                    tag="control_target",
                )
                dpg.add_line_series(
                    x=self.data_x,
                    y=self.feedback_y,
                    label="Feedback",
                    parent="y_axis",
                    tag="control_feedback",
                )

    def update_data(self, x, feedback_y) -> None:
        """Update the data."""
        self.data_x.append(x)
        self.target_y.append(self.inputs["target"].value)
        self.feedback_y.append(feedback_y)

        dpg.set_value(
            "control_target",
            [
                list(self.data_x[-self.n_samples :]),
                list(self.target_y[-self.n_samples :]),
            ],
        )
        dpg.set_value(
            "control_feedback",
            [
                list(self.data_x[-self.n_samples :]),
                list(self.feedback_y[-self.n_samples :]),
            ],
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
