from __future__ import annotations
import itertools
import dearpygui.dearpygui as dpg
from typing import Callable


class Widget:
    """Create a dearpygui widget."""

    general_callback: callable

    def __init__(self, label: str | Callable[[], str]) -> None:
        self.label = label
        self.enabled: bool
        self.tag = dpg.generate_uuid()

    def callback(self) -> None:
        """Link to the general callback with objects callback string."""
        Widget.general_callback(self.label)

    def create(self) -> None:
        """Create the widget."""

    def update(self, enabled: bool) -> None:
        """Update the widget."""


class Text(Widget):
    """Create a dearpygui text."""

    def __init__(self, pre_label: str, label: Callable[[], str] = "") -> None:
        super().__init__(label)
        self.pre_label = pre_label

    def create(self) -> None:
        """Create the widget."""
        dpg.add_text(tag=self.tag)

    def update(self, enabled: bool) -> None:
        """Update the widget."""
        value = self.label if isinstance(self.label, str) else self.label()
        dpg.set_value(self.tag, self.pre_label + value)


class Button(Widget):
    """Create a dearpygui button."""

    def __init__(self, label: str) -> None:
        super().__init__(label)

    def create(self) -> None:
        """Create the widget."""
        dpg.add_button(label=self.label, callback=self.callback, tag=self.tag)

    def update(self, enabled: bool) -> None:
        """Update the widget."""
        dpg.configure_item(self.tag, enabled=enabled)


class Checkbox(Widget):
    """Create a dearpygui button."""

    def __init__(self, label: str, value: Callable[[], bool]) -> None:
        self.value = value
        super().__init__(label)

    def create(self) -> None:
        """Create the widget."""
        dpg.add_checkbox(label=self.label, callback=self.callback, tag=self.tag)

    def update(self, enabled: bool) -> None:
        """Update the widget."""
        value = self.value if isinstance(self.value, bool) else self.value()
        dpg.configure_item(self.tag, enabled=enabled)
        dpg.set_value(self.tag, value)


class Group:
    """A group of widgets."""

    groups: list["Group"] = []

    @staticmethod
    def update_all() -> None:
        """Update all groups."""
        for group in Group.groups:
            group.update()

    def __init__(
        self, widgets: list["Widget"], enabled: bool | Callable[[], bool] = False
    ) -> None:
        self.widgets = widgets
        self.enabled = enabled
        Group.groups.append(self)

    def update(self) -> None:
        """Update the group."""
        enabled = self.enabled if isinstance(self.enabled, bool) else self.enabled()
        for widget in self.widgets:
            widget.update(enabled)


class Row(Group):
    """A row of widgets."""

    def __init__(
        self, widgets: list[Widget], enabled: bool | Callable[[], bool] = False
    ) -> None:
        super().__init__(widgets, enabled)
        with dpg.group(horizontal=True):
            for widget in widgets:
                widget.create()
        self.update()


class Table(Group):
    """A table of widgets."""

    def __init__(
        self,
        headers: list[str] | None,
        widgets: list[list[Widget]],
        enabled: bool | Callable[[], bool] = False,
    ) -> None:
        super().__init__(list(itertools.chain(*widgets)), enabled)
        header_row = headers is not None
        with dpg.table(header_row=header_row, borders_outerH=True, borders_outerV=True):
            for n in range(len(widgets[0])):
                header = None if headers is None else headers[n]
                dpg.add_table_column(label=header)
            for row in widgets:
                with dpg.table_row():
                    for widget in row:
                        widget.create()
        self.update()


def window(
    width: int = 0, height: int = 0, pos: list = None, tag: int = None
) -> dpg.contextmanager:
    """Create a dearpygui window."""
    if not tag:
        tag = dpg.generate_uuid()
    return dpg.window(
        no_move=True,
        no_resize=True,
        no_collapse=True,
        no_close=True,
        no_title_bar=True,
        no_bring_to_front_on_focus=True,
        no_scrollbar=True,
        no_scroll_with_mouse=True,
        width=width,
        height=height,
        min_size=[width, height],
        max_size=[width, height],
        pos=pos,
        tag=tag,
    )


def create_plot(
    label: str, width: int, height: int, pos: list, names: list, limit: float
) -> None:
    """Create a dearpygui bar plot."""
    with window(width, height, pos), dpg.plot(height=-1, width=-1):
        tag = dpg.generate_uuid()
        dpg.add_plot_axis(
            dpg.mvXAxis,
            label=label,
            no_gridlines=True,
            no_tick_labels=True,
            no_tick_marks=True,
            tag=tag,
        )
        dpg.set_axis_limits(tag, 0, len(names))

        tag = dpg.generate_uuid()
        dpg.add_plot_axis(dpg.mvYAxis, tag=tag)
        dpg.set_axis_limits(tag, -limit, limit)

        for x, name in enumerate(names):
            dpg.add_bar_series([0.5 + x], [0], weight=0.8, parent=tag, tag=label + name)


def update_plot(label: str, name: str, value: float) -> None:
    """Update a dearpygui bar plot."""
    data = dpg.get_value(label + name)
    data[1] = [value]
    dpg.set_value(label + name, data)
