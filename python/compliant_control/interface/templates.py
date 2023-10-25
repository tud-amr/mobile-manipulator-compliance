import dearpygui.dearpygui as dpg


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


def button(label: str, enabled: bool = False, callback: callable = None) -> None:
    """Create a dearpygui button."""
    dpg.add_button(label=label, enabled=enabled, callback=callback, tag=label)


def checkbox(
    label: str, enabled: bool = False, callback: callable = None, tag: str = None
) -> None:
    """Create a dearpygui checkbox."""
    if label is not None:
        tag = label
    dpg.add_checkbox(label=label, default_value=enabled, callback=callback, tag=tag)


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
