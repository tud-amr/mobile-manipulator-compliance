import dearpygui.dearpygui as dpg
import numpy as np


class Wheel:
    direction = 0
    x = 0
    y = 0

    def __init__(self, name: str) -> None:
        self.name = name
        self.start_position: float = None
        self.position: float = 0
        self.speed: float = 0
        self.voltage: float = 0
        self.current: float = 0

    @property
    def command(self) -> None:
        """Get the command."""
        direction = 0.0
        gain = 0.15
        threshold = 0.25

        movement: str
        angle = -np.arctan2(Wheel.y, Wheel.x)
        if -np.pi * 7 / 8 <= angle <= -np.pi * 5 / 8:
            movement = "W"
        elif -np.pi * 5 / 8 <= angle <= -np.pi * 3 / 8:
            movement = "Z"
        elif -np.pi * 3 / 8 <= angle <= -np.pi * 1 / 8:
            movement = "ZO"
        elif -np.pi * 1 / 8 <= angle <= np.pi * 1 / 8:
            movement = "O"
        elif np.pi * 1 / 8 <= angle <= np.pi * 3 / 8:
            movement = "NO"
        elif np.pi * 3 / 8 <= angle <= np.pi * 5 / 8:
            movement = "N"
        elif np.pi * 5 / 8 <= angle <= np.pi * 7 / 8:
            movement = "W"
        else:
            movement = "W"

        if angle > 0:
            movement = "N"
        elif angle < 0:
            movement = "Z"

        match movement:
            case "N":
                direction = -1
            case "Z":
                direction = 1
            case "O":
                if self.name in ["front_right_wheel", "rear_left_wheel"]:
                    direction = -1
                else:
                    direction = 1
            case "W":
                if self.name in ["front_right_wheel", "rear_left_wheel"]:
                    direction = 1
                else:
                    direction = -1
            case "ZW":
                if self.name in ["front_right_wheel", "rear_left_wheel"]:
                    direction = 1
            case "NW":
                if self.name in ["front_left_wheel", "rear_right_wheel"]:
                    direction = -1
            case "ZO":
                if self.name in ["front_left_wheel", "rear_right_wheel"]:
                    direction = 1
            case "NO":
                if self.name in ["front_right_wheel", "rear_left_wheel"]:
                    direction = -1

        if "left" in self.name:
            direction *= -1
        if movement in ["NO", "NW", "ZO", "ZW"]:
            gain += 0.05
        if np.linalg.norm([Wheel.x, Wheel.y]) < threshold:
            gain = 0.0
        return direction * gain


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
        self.w_win = 400
        self.h_win = 900
        self.w_plt = int(self.w_win / 2)
        self.h_plt = int(self.h_win / 3)

    def create_ui(self) -> None:
        """Create the ui."""
        self.active = True
        dpg.create_context()
        dpg.create_viewport(
            title="Dingo Controller",
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
        self.create_drawing([0, 2 * self.h_plt])

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

    def create_drawing(self, pos: list) -> None:
        """Create a drawing."""
        with dpg.window(
            width=self.w_win,
            height=self.h_plt,
            pos=pos,
            no_resize=True,
            no_move=True,
            no_close=True,
            no_collapse=True,
            no_title_bar=True,
            no_bring_to_front_on_focus=True,
            no_scrollbar=True,
            no_scroll_with_mouse=True,
        ):
            with dpg.drawlist(width=self.w_win, height=self.h_plt, tag="drawing"):
                pass

    def update_drawing(self) -> None:
        if dpg.does_item_exist(item="arrow_layer"):
            dpg.delete_item(item="arrow_layer")
        dpg.add_draw_layer(tag="arrow_layer", parent="drawing")
        center_x = int(self.w_win / 2)
        center_y = int(self.h_plt / 2)
        point_x = center_x - Wheel.x * 50
        point_y = center_y + Wheel.y * 50
        dpg.draw_arrow(
            (point_x, point_y),
            (center_x, center_y),
            color=(0, 200, 255),
            parent="arrow_layer",
            thickness=5,
            size=5,
        )

    def update_feedback(self) -> None:
        """Update the feedback."""
        Wheel.direction = 0
        Wheel.x = 0
        Wheel.y = 0
        for wheel in self.wheels:
            for data_name in self.data_names:
                data = dpg.get_value(f"{wheel.name}_{data_name}")
                data[1] = [getattr(wheel, data_name)]
                dpg.set_value(f"{wheel.name}_{data_name}", data)
                Wheel.direction += wheel.speed
                if wheel.name == "front_left_wheel":
                    Wheel.x += wheel.speed
                    Wheel.y += wheel.speed
                elif wheel.name == "front_right_wheel":
                    Wheel.x -= wheel.speed
                    Wheel.y += wheel.speed
                elif wheel.name == "rear_left_wheel":
                    Wheel.x -= wheel.speed
                    Wheel.y += wheel.speed
                elif wheel.name == "rear_right_wheel":
                    Wheel.x += wheel.speed
                    Wheel.y += wheel.speed
        self.update_drawing()

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
