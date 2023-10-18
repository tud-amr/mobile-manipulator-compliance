class ControlModeInformation:
    """https://github.com/Kinovarobotics/kortex/blob/master/api_python/doc/markdown/messages/ActuatorConfig/ControlModeInformation.md."""

    def __init__(self, control_mode: int = 0) -> None:
        self.control_mode: int = control_mode


class CommandModeInformation:
    """https://github.com/Kinovarobotics/kortex/blob/master/api_python/doc/markdown/messages/ActuatorConfig/CommandModeInformation.md."""

    def __init__(self, command_mode: int = None) -> None:
        self.command_mode: int = command_mode
