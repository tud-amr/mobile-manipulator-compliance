class Feedback:
    """https://github.com/Kinovarobotics/kortex/blob/master/api_python/doc/markdown/messages/BaseCyclic/Feedback.md."""

    def __init__(self, n: int) -> None:
        self.actuators = [
            ActuatorFeedback(position=0, velocity=0, torque=0, current_motor=0)
            for _ in range(n)
        ]


class ActuatorFeedback:
    """https://github.com/Kinovarobotics/kortex/blob/master/api_python/doc/markdown/messages/BaseCyclic/ActuatorFeedback.md."""

    def __init__(
        self,
        position: float = None,
        velocity: float = None,
        torque: float = None,
        current_motor: float = None,
        temperature_motor: float = None,
        temperature_core: float = None,
    ) -> None:
        self.position: float = position
        self.velocity: float = velocity
        self.torque: float = torque
        self.current_motor: float = current_motor
        self.temperature_motor: float = temperature_motor
        self.temperature_core: float = temperature_core


class Command:
    """https://github.com/Kinovarobotics/kortex/blob/master/api_python/doc/markdown/messages/BaseCyclic/Command.md."""

    def __init__(
        self, frame_id: int, actuators: list["ActuatorCommand"], interconnect: None
    ) -> None:
        self.frame_id: int = frame_id
        self.actuators: list["ActuatorCommand"] = actuators
        self.interconnect: None = interconnect


class ActuatorCommand:
    """https://github.com/Kinovarobotics/kortex/blob/master/api_python/doc/markdown/messages/BaseCyclic/ActuatorCommand.md."""

    def __init__(
        self,
        command_id: int = None,
        flags: int = None,
        position: float = None,
        velocity: float = None,
        torque_joint: float = None,
        current_motor: float = None,
    ) -> None:
        self.command_id = command_id
        self.flags = flags
        self.position: float = position
        self.velocity: float = velocity
        self.torque_joint: float = torque_joint
        self.current_motor: float = current_motor
