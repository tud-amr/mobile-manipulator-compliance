import dearpygui.dearpygui as dpg
from compliant_control.interface.templates import window
import numpy as np


class Joystick:
    """Defines a dearpygui joystick."""

    def __init__(self, width: int, height: int, pos: list, callback: callable) -> None:
        self.w_win = width
        self.h_win = height
        self.pos = pos
        self.callback = callback

        self.point_x = 0
        self.point_y = 0
        self.create_drawing()
        self.update_drawing()

    @property
    def direction(self) -> list:
        """Return the direction as vector."""
        return [self.point_x, -self.point_y]

    def create_drawing(self) -> None:
        """Create a drawing."""
        with dpg.handler_registry():
            dpg.add_mouse_down_handler(callback=self.mouse_down)
            dpg.add_mouse_release_handler(callback=self.mouse_release)

        with window(self.w_win, self.h_win, self.pos):
            with dpg.drawlist(width=self.w_win, height=self.h_win, tag="drawing"):
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
        self.callback()

    def mouse_down(self) -> None:
        """Handle mouse down input."""
        if dpg.is_item_focused("drawing"):
            x, y = np.array(dpg.get_mouse_pos())
            x -= self.w_win / 2
            y -= self.h_win / 2
            self.point_x = x / np.linalg.norm([x, y])
            self.point_y = y / np.linalg.norm([x, y])
            self.update_drawing()
        self.callback()
