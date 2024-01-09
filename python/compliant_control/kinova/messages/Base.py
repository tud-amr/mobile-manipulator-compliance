from __future__ import annotations

class ServoingModeInformation:
    """https://github.com/Kinovarobotics/kortex/blob/master/api_python/doc/markdown/messages/Base/ServoingModeInformation.md."""

    def __init__(self, servoing_mode: int = None) -> None:
        self.servoing_mode: int = servoing_mode


class ControlModeInformation:
    """https://github.com/Kinovarobotics/kortex/blob/master/api_python/doc/markdown/messages/Base/ControlModeInformation.md."""

    def __init__(self, mode: int = None) -> None:
        self.mode: int = mode


class OperatingModeInformation:
    """https://github.com/Kinovarobotics/kortex/blob/master/api_python/doc/markdown/messages/Base/OperatingModeInformation.md."""

    def __init__(self, operating_mode: int = None) -> None:
        self.operating_mode: int = operating_mode


class ActuatorInformation:
    """https://github.com/Kinovarobotics/kortex/blob/master/api_python/doc/markdown/messages/Base/ActuatorInformation.md."""

    def __init__(self, count: int = None) -> None:
        self.count: int = count


class ActionList:
    """https://github.com/Kinovarobotics/kortex/blob/master/api_python/doc/markdown/messages/Base/ActionList.md."""

    def __init__(self, action_list: list["Action"]) -> None:
        self.action_list: list[Action] = action_list


class Action:
    """https://github.com/Kinovarobotics/kortex/blob/master/api_python/doc/markdown/messages/Base/Action.md."""

    def __init__(self, name: str, handle: "ActionHandle") -> None:
        self.name: str = name
        self.handle: ActionHandle = handle


class ActionHandle:
    """https://github.com/Kinovarobotics/kortex/blob/master/api_python/doc/markdown/messages/Base/ActionHandle.md."""

    def __init__(self, identifier: int, action_type: int, permission: int) -> None:
        self.identifier: int = identifier
        self.action_type: int = action_type
        self.permission: int = permission


class RequestedActionType:
    """https://github.com/Kinovarobotics/kortex/blob/master/api_python/doc/markdown/messages/Base/RequestedActionType.md."""

    def __init__(self, action_type: int) -> None:
        self.action_type: int = action_type


class NotificationOptions:
    """https://github.com/Kinovarobotics/kortex/blob/master/api_python/doc/markdown/messages/Common/NotificationOptions.md."""

    def __init__(self, type: int, rate_m_sec: int, threshold_value: float) -> None:
        self.type: int = type
        self.rate_m_sec: int = rate_m_sec
        self.threshold_value: float = threshold_value


class NotificationHandle:
    """https://github.com/Kinovarobotics/kortex/blob/master/api_python/doc/markdown/messages/Common/NotificationHandle.md."""

    def __init__(self, identifier: int) -> None:
        self.identifier: int = identifier
