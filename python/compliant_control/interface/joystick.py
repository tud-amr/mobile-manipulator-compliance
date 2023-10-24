import dearpygui.dearpygui as dpg
import numpy as np
from compliant_control.interface.callbacks import Callbacks


class Joystick:
    """Defines a dearpygui joystick."""

    def __init__(
        self, width: int, height: int, pos: list, callbacks: Callbacks
    ) -> None:
        self.w_win = width
        self.h_win = height
        self.pos = pos
        self.callbacks = callbacks
        self.point_x = 0
        self.point_y = 0
        self.create_drawing()
        self.update_drawing()

    def create_drawing(self) -> None:
        """Create a drawing."""
        with dpg.handler_registry():
            dpg.add_mouse_down_handler(callback=self.mouse_down)
            dpg.add_mouse_release_handler(callback=self.mouse_release)

        with dpg.window(
            width=self.w_win,
            height=self.h_win,
            pos=self.pos,
            no_resize=True,
            no_move=True,
            no_close=True,
            no_collapse=True,
            no_title_bar=True,
            no_bring_to_front_on_focus=True,
            no_scrollbar=True,
            no_scroll_with_mouse=True,
        ), dpg.drawlist(width=self.w_win, height=self.h_win, tag="drawing"):
            pass

    def update_drawing(self) -> None:
        """Update the drawing."""
        if dpg.does_item_exist(item="draw_layer"):
            dpg.delete_item(item="draw_layer")
        dpg.add_draw_layer(tag="draw_layer", parent="drawing")
        center_x = int(self.w_win / 2)
        center_y = int(self.h_win / 2)
        point_x = center_x + self.point_x * 100
        point_y = center_y + self.point_y * 100
        dpg.draw_circle(
            (center_x, center_y),
            5,
            parent="draw_layer",
            color=(0, 200, 255),
            fill=(0, 200, 255),
        )
        dpg.draw_arrow(
            (point_x, point_y),
            (center_x, center_y),
            color=(0, 200, 255),
            parent="draw_layer",
            thickness=5,
            size=5,
        )

    def mouse_release(self) -> None:
        """Handle mouse release input."""
        self.point_x = 0
        self.point_y = 0
        self.update_drawing()
        self.callbacks.joystick([self.point_x, self.point_y])

    def mouse_down(self) -> None:
        """Handle mouse down input."""
        if dpg.is_item_focused("drawing"):
            x, y = np.array(dpg.get_mouse_pos())
            x -= self.w_win / 2
            y -= self.h_win / 2
            self.point_x = x / np.linalg.norm([x, y])
            self.point_y = y / np.linalg.norm([x, y])
            self.update_drawing()
        self.callbacks.joystick([self.point_x, -self.point_y])
