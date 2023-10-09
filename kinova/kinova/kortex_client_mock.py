from .kortex_client import KortexClient
from .messages import Base, BaseCyclic, ActuatorConfig
from compliant_controller.state import State


class KortexClientMock(KortexClient):
    """A mock of the Kortex Client class, to make testing without robot possible."""

    def __init__(self, state: State, n_joints: int) -> None:
        base = BaseClientMock(n_joints)
        base_cyclic = BaseCyclicClientMock(n_joints)
        actuator_config = ActuatorConfigClientMock(n_joints)
        super().__init__(state, base, base_cyclic, actuator_config, mock=True)


class BaseClientMock:
    """https://github.com/Kinovarobotics/kortex/blob/master/api_python/doc/markdown/summary_pages/Base.md."""

    def __init__(self, n: int) -> None:
        self.n = n
        self.servoing_mode = Base.ServoingModeInformation(0)

    def GetServoingMode(self) -> Base.ServoingModeInformation:
        """Retrieves current servoing mode."""
        return self.servoing_mode

    def SetServoingMode(
        self, servoing_mode_information: Base.ServoingModeInformation
    ) -> None:
        """Sets the servoing mode."""
        self.servoing_mode.servoing_mode = servoing_mode_information.servoing_mode

    def GetControlMode(self) -> Base.ControlModeInformation:
        """Retrieves current control mode. NOTE: Seems not to work."""
        return Base.ControlModeInformation()

    def GetOperatingMode(self) -> Base.OperatingModeInformation:
        """Retrieves current operating mode."""
        return Base.OperatingModeInformation()

    def GetActuatorCount(self) -> Base.ActuatorInformation:
        """Retrieves the number of actuators in the robot."""
        return Base.ActuatorInformation(self.n)

    def ReadAllActions(
        self, request_action_type: Base.RequestedActionType
    ) -> Base.ActionList:
        """Retrieves a list of all existing actions."""
        return Base.ActionList([Base.Action("Home", Base.ActionHandle(0, 0, 0))])

    def OnNotificationActionTopic(
        self, callback: callable, notification_options: Base.NotificationOptions
    ) -> Base.NotificationHandle:
        """Subscribes to action topic for notifications."""
        return Base.NotificationHandle(0)

    def ExecuteActionFromReference(self, action_handle: Base.ActionHandle) -> None:
        """Commands the robot to execute the specified existing action."""

    def Unsubscribe(self, notification_handle: Base.NotificationHandle) -> None:
        """Unsubscribes client from receiving notifications for the specified topic."""


class BaseCyclicClientMock:
    """https://github.com/Kinovarobotics/kortex/blob/master/api_python/doc/markdown/summary_pages/BaseCyclic.md."""

    def __init__(self, n: int) -> None:
        self.n = n
        self.feedback = BaseCyclic.Feedback(self.n)

    def Refresh(self, command: BaseCyclic.Command) -> BaseCyclic.Feedback:
        """Sends a command to actuators and interface and returns feedback from base, actuators, and interface on actual status."""
        return self.feedback

    def RefreshFeedback(self) -> BaseCyclic.Feedback:
        """Obtains feedback from base, actuators, and interface on their status."""
        return self.feedback


class ActuatorConfigClientMock:
    """https://github.com/Kinovarobotics/kortex/blob/master/api_python/doc/markdown/summary_pages/ActuatorConfig.md."""

    def __init__(self, n: int) -> None:
        self.n = n
        self.control_modes = [
            ActuatorConfig.ControlModeInformation(1) for _ in range(self.n)
        ]

    def GetControlMode(self, device_id: int) -> ActuatorConfig.ControlModeInformation:
        """Retrieves actuator control mode."""
        return self.control_modes[device_id]

    def GetCommandMode(self) -> ActuatorConfig.CommandModeInformation:
        """Retrieves command mode (config versus cyclic)."""
        return ActuatorConfig.CommandModeInformation()

    def SetControlMode(
        self,
        control_mode_information: ActuatorConfig.ControlModeInformation,
        device_id: int,
    ) -> None:
        """Sets actuator control mode."""
        self.control_modes[
            device_id
        ].control_mode = control_mode_information.control_mode
