import time
import numpy as np
from kortex_api.autogen.messages import ActuatorConfig_pb2
from compliant_control.kinova.kortex_client import KortexClient
from compliant_control.kinova.messages import Base, BaseCyclic, ActuatorConfig
from compliant_control.kinova.specifications import Position

from compliant_control.control.state import State
from compliant_control.mujoco.simulation import Simulation

JOINTS = 6


class KortexClientSimulation(KortexClient):
    """A simulation of the Kortex Client class."""

    def __init__(self, state: State, simulation: Simulation) -> None:
        self.base = BaseClientSimulation()
        self.simulation = simulation
        self.base_cyclic = BaseCyclicClientSimulation(state, simulation)
        self.actuator_config = ActuatorConfigClientSimulation(simulation)
        super().__init__(
            state=state,
            base=self.base,
            base_cyclic=self.base_cyclic,
            actuator_config=self.actuator_config,
            simulate=True,
        )

    def _high_level_move(self, position: Position) -> None:
        max_attempts = 100
        divider = 100
        moving_speed = 20  # deg/s
        step_deg = moving_speed / self.frequency
        step_size = np.deg2rad(step_deg)

        pose = np.deg2rad(position.position)
        reached = np.full(self.actuator_count, False)
        no_improve = np.zeros(self.actuator_count)
        min_error = np.absolute(pose - self.state.kinova_feedback.q)
        while not reached.all():
            error = pose - self.state.kinova_feedback.q
            abs_error = np.absolute(error)
            improved = abs_error < min_error
            min_error = np.where(improved, abs_error, min_error)
            no_improve += np.invert(improved)
            no_improve *= np.invert(improved)
            reached = no_improve > max_attempts
            increment = np.minimum(abs_error / divider, step_size)
            self.simulation.ctrl_increment(np.sign(error) * increment)
            time.sleep(1 / self.frequency)


class BaseClientSimulation:
    """https://github.com/Kinovarobotics/kortex/blob/master/api_python/doc/markdown/summary_pages/Base.md."""

    def __init__(self) -> None:
        self.servoing_mode = Base.ServoingModeInformation(0)

    def GetServoingMode(self) -> Base.ServoingModeInformation:
        """Retrieve current servoing mode."""
        return self.servoing_mode

    def SetServoingMode(
        self, servoing_mode_information: Base.ServoingModeInformation
    ) -> None:
        """Set the servoing mode."""
        self.servoing_mode.servoing_mode = servoing_mode_information.servoing_mode

    def GetActuatorCount(self) -> Base.ActuatorInformation:
        """Retrieve the number of actuators in the robot."""
        return Base.ActuatorInformation(JOINTS)


class BaseCyclicClientSimulation:
    """https://github.com/Kinovarobotics/kortex/blob/master/api_python/doc/markdown/summary_pages/BaseCyclic.md."""

    def __init__(self, state: State, simulation: Simulation) -> None:
        self.state = state
        self.sim = simulation
        self.feedback = BaseCyclic.Feedback(JOINTS)

    def Refresh(self, command: BaseCyclic.Command) -> BaseCyclic.Feedback:
        """Send a command to actuators and interface and returns feedback from base, actuators, and interface on actual status."""
        self.sim.set_ctrl_value(
            "Kinova",
            "position",
            [command.actuators[n].position for n in range(JOINTS)],
        )
        self.sim.set_ctrl_value(
            "Kinova",
            "velocity",
            [command.actuators[n].velocity for n in range(JOINTS)],
        )
        self.sim.set_ctrl_value(
            "Kinova",
            "torque",
            [command.actuators[n].current_motor for n in range(JOINTS)],
        )
        return self.RefreshFeedback()

    def RefreshFeedback(self) -> BaseCyclic.Feedback:
        """Obtain feedback from base, actuators, and interface on their status."""
        for n, pos in enumerate(self.sim.get_sensor_feedback("Kinova", "position")):
            self.feedback.actuators[n].position = pos
        for n, vel in enumerate(self.sim.get_sensor_feedback("Kinova", "velocity")):
            self.feedback.actuators[n].velocity = vel
        for n, tor in enumerate(self.sim.get_sensor_feedback("Kinova", "torque")):
            self.feedback.actuators[n].current_motor = tor
        return self.feedback


class ActuatorConfigClientSimulation:
    """https://github.com/Kinovarobotics/kortex/blob/master/api_python/doc/markdown/summary_pages/ActuatorConfig.md."""

    def __init__(self, simulation: Simulation) -> None:
        self.simulation = simulation
        self.control_modes = [
            ActuatorConfig.ControlModeInformation(1) for _ in range(JOINTS)
        ]

    def GetControlMode(self, device_id: int) -> ActuatorConfig.ControlModeInformation:
        """Retrieve actuator control mode."""
        return self.control_modes[device_id]

    def SetControlMode(
        self,
        control_mode_information: ActuatorConfig.ControlModeInformation,
        device_id: int,
    ) -> None:
        """Set actuator control mode."""
        mode_value = control_mode_information.control_mode
        if mode_value == ActuatorConfig_pb2.POSITION:
            self.simulation.change_mode("position", device_id)
        elif mode_value == ActuatorConfig_pb2.CURRENT:
            self.simulation.change_mode("torque", device_id)
        self.control_modes[device_id].control_mode = mode_value
