import time
from threading import Thread, Event
from typing import TYPE_CHECKING, Literal
import numpy as np

from kortex_api.RouterClient import RouterClient
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.client_stubs.ActuatorConfigClientRpc import ActuatorConfigClient
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, ActuatorConfig_pb2
from kortex_api.Exceptions.KServerException import KServerException

from user_interface.logger import Logger
from .specifications import actuator_ids, ranges

if TYPE_CHECKING:
    from compliant_controller.controller import Controller
    from compliant_controller.state import State


class KortexClient:
    """Class that uses the Kortex API to communicate with the robot."""

    def __init__(
        self,
        state: "State",
        base: BaseClient = None,
        base_cyclic: BaseCyclicClient = None,
        actuator_config: ActuatorConfigClient = None,
        router: RouterClient = None,
        real_time_router: RouterClient = None,
        mock: bool = False,
        simulate: bool = False,
    ) -> None:
        if None in [base, base_cyclic, actuator_config]:
            self.base = BaseClient(router)
            self.base_cyclic = BaseCyclicClient(real_time_router)
            self.actuator_config = ActuatorConfigClient(router)
        else:
            self.base = base
            self.base_cyclic = base_cyclic
            self.actuator_config = actuator_config
        self.mock = mock
        self.simulate = simulate
        self.time_out_duration = 3 if self.mock else 20
        self.actuator_count = self.base.GetActuatorCount().count
        self.active = False
        self.changing_servoing_mode = False
        self.controller_connected = False
        self.active_loop = False

        self.frequency = 1000
        self.rate = self.frequency
        self.n = self.frequency
        self.sleep_time = 1 / self.frequency

        self.state = state
        self.state.connect_client(self)
        self._set_servoing_mode(Base_pb2.SINGLE_LEVEL_SERVOING)
        self._refresh()
        self._initialize_command()
        self.start_refresh_loop()

    @property
    def low_level_control(self) -> bool:
        """Returns whether low_level_control is active."""
        return self.servoing_mode == Base_pb2.LOW_LEVEL_SERVOING

    def HLC_available(self) -> bool:
        """Return whether mode is HLC and available."""
        return not self.active and not self.low_level_control

    def LLC_available(self) -> bool:
        """Return whether LLC is available to connect controller."""
        return (
            not self.active and self.low_level_control and not self.controller_connected
        )

    def LLC_connected(self) -> bool:
        """Return whether a LLC controller is connected."""
        return not self.active and self.controller_connected

    def get_actuator_state(self, n: int, key: str) -> float:
        """Get the requested state for a given actuator."""
        return getattr(self.feedback.actuators[n], key)

    def get_control_mode(self, joint: int) -> str:
        """Get the control mode of an actuator."""
        return ActuatorConfig_pb2.ControlMode.Name(self.actuator_modes[joint])

    def get_servoing_mode(self) -> str:
        """Get the servoing mode of the robot."""
        return Base_pb2.ServoingMode.Name(self.servoing_mode)

    def get_update_rate(self) -> int:
        """Get the update rate."""
        return self.rate

    def clear_faults(self) -> None:
        """Clear the faults."""
        self.base.ClearFaults()

    def start_refresh_loop(self) -> None:
        """Start the refresh loop."""
        self.refresh_thread = Thread(target=self._refresh_loop)
        self.rate_check_thread = Thread(target=self._rate_check_loop)
        self.active_loop = True
        self.refresh_thread.start()
        self.rate_check_thread.start()

    def stop_refresh_loop(self) -> None:
        """Stop the update loop."""
        self.active_loop = False

    def set_control_mode(
        self, joint: int, mode: Literal["position", "velocity", "current"]
    ) -> None:
        """Set the control mode of an actuator."""
        mode = getattr(ActuatorConfig_pb2, mode.upper())
        control_mode_information = ActuatorConfig_pb2.ControlModeInformation()
        control_mode_information.control_mode = mode
        _id = joint if self.mock else actuator_ids[joint]
        self.actuator_config.SetControlMode(control_mode_information, _id)
        self._update_modes()

    def start_LLC(self) -> None:
        """Start low_level control."""
        self._start_control(self._start_LLC)

    def stop_LLC(self) -> None:
        """Stop low_level control."""
        self._start_control(self._stop_LLC)

    def connect_LLC(
        self, controller: "Controller", mode: Literal["position", "velocity", "current"]
    ) -> None:
        """Connect a controller to the LLC of the robot."""
        self._start_control(self._connect_LLC, [controller, mode])

    def disconnect_LLC(self) -> None:
        """Disconnect a controller from the LLC of the robot."""
        self._start_control(self._disconnect_LLC)

    def home(self) -> bool:
        """Move the arm to the home position."""
        self._start_control(self._action, ["Home"])

    def zero(self) -> bool:
        """Move the arm to the zero position."""
        self._start_control(self._action, ["Zero"])

    def retract(self) -> bool:
        """Move the arm to the retract position."""
        self._start_control(self._action, ["Retract"])

    def get_position(self, joint: int, as_percentage: bool) -> float:
        """Get the position of a joint."""
        position = getattr(self.feedback.actuators[joint], "position")
        if self.mock:
            position = np.rad2deg(position)
        lower_bound = ranges["position"][joint][0]
        upper_bound = ranges["position"][joint][1]
        position -= 360 if position > upper_bound else 0
        if as_percentage:
            return (position - lower_bound) / (upper_bound - lower_bound)
        return np.deg2rad(position)

    def get_velocity(self, joint: int, as_percentage: bool) -> float:
        """Get the velocity of a joint."""
        velocity = getattr(self.feedback.actuators[joint], "velocity")
        if self.mock:
            velocity = np.rad2deg(velocity)
        if as_percentage:
            lower_bound = ranges["velocity"][joint][0]
            upper_bound = ranges["velocity"][joint][1]
            return (abs(velocity) - lower_bound) / (upper_bound - lower_bound)
        return np.deg2rad(velocity)

    def get_current(self, joint: int, as_percentage: bool) -> float:
        """Get the current of a joint."""
        current = getattr(self.feedback.actuators[joint], "current_motor")
        if as_percentage:
            lower_bound = ranges["current_motor"][joint][0]
            upper_bound = ranges["current_motor"][joint][1]
            return (abs(current) - lower_bound) / (upper_bound - lower_bound)
        return current

    def get_torque(self, joint: int, as_percentage: bool) -> float:
        """Get the torque of a joint."""
        torque = getattr(self.feedback.actuators[joint], "torque")
        if as_percentage:
            lower_bound = ranges["torque"][joint][0]
            upper_bound = ranges["torque"][joint][1]
            return (abs(torque) - lower_bound) / (upper_bound - lower_bound)
        return getattr(self.feedback.actuators[joint], "torque")

    def _start_LLC(self) -> None:
        """Start low_level control."""
        for n in range(self.actuator_count):
            self._copy_feedback_to_command_message(n)
            self.set_control_mode(n, "position")
        self._set_servoing_mode(Base_pb2.LOW_LEVEL_SERVOING)
        Logger.log("Low_level control enabled.")

    def _stop_LLC(self) -> None:
        """Stop low_level control."""
        self._set_servoing_mode(Base_pb2.SINGLE_LEVEL_SERVOING)
        Logger.log("Low_level control disabled.")

    def _connect_LLC(self, controller: "Controller", mode: str) -> None:
        """Connect a controller to the LLC of the robot."""
        self.controller = controller
        for joint in controller.joints:
            self._copy_feedback_to_command_message(joint)
            self.base_cyclic.Refresh(self.command)
            self.set_control_mode(joint, mode)
        self.controller_connected = True
        Logger.log("Controller connected.")

    def _disconnect_LLC(self) -> None:
        """Disconnect a controller from the LLC of the robot."""
        self.controller_connected = False
        for joint in range(self.actuator_count):
            self.set_control_mode(joint, "position")
        Logger.log("Controller disconnected.")

    def _copy_commands_to_command_message(self) -> None:
        self.controller.command()
        for joint, command in zip(self.controller.joints, self.controller.commands):
            if self.controller.mode == "current":
                self.command.actuators[joint].current_motor = command
            else:
                command = command if self.mock else np.rad2deg(command)
                self.command.actuators[joint].position = command

    def _refresh_loop(self) -> bool:
        while self.active_loop:
            self._refresh()
            self.state.update()
            self.n += 1
            time.sleep(self.sleep_time)

    def _refresh(self) -> None:
        """Refresh."""
        if not self.changing_servoing_mode and self.controller_connected:
            for joint in self.controller.joints:
                self._copy_feedback_to_command_message(joint)
            self._copy_commands_to_command_message()
            try:
                self.feedback = self.base_cyclic.Refresh(self.command)
            except KServerException:
                Logger.log("Robot control lost.")
                self.controller_connected = False
        else:
            self.feedback = self.base_cyclic.RefreshFeedback()

    def _rate_check_loop(self) -> None:
        """Define te rate check loop."""
        while self.active_loop:
            self.rate = self.n
            self.n = 0
            self.sleep_time *= self.rate / self.frequency
            time.sleep(1)

    def _set_servoing_mode(self, value: int) -> None:
        """Set the servoing mode of the robot."""
        self.changing_servoing_mode = True
        base_servo_mode = Base_pb2.ServoingModeInformation()
        base_servo_mode.servoing_mode = value
        self.base.SetServoingMode(base_servo_mode)
        self._update_modes()
        self.changing_servoing_mode = False

    def _update_modes(self) -> None:
        """Update the modes."""
        self.servoing_mode = self.base.GetServoingMode().servoing_mode
        actuator_modes = []
        for n in range(self.actuator_count):
            _id = n if self.mock else actuator_ids[n]
            actuator_modes.append(self.actuator_config.GetControlMode(_id).control_mode)
        self.actuator_modes = actuator_modes

    def _initialize_command(self) -> None:
        self.command = BaseCyclic_pb2.Command()
        for n in range(self.actuator_count):
            actuator_command = BaseCyclic_pb2.ActuatorCommand()
            actuator_command.flags = 1
            actuator_command.position = self.feedback.actuators[n].position
            actuator_command.velocity = self.feedback.actuators[n].velocity
            self.command.actuators.extend([actuator_command])

    def _copy_feedback_to_command_message(self, joint: int) -> None:
        self.command.actuators[joint].position = self.feedback.actuators[joint].position
        if not self.mock:
            self.command.actuators[joint].current_motor = self.feedback.actuators[
                joint
            ].current_motor

    def _action(self, name: str) -> bool:
        """Perform the provided action."""
        self._set_servoing_mode(Base_pb2.SINGLE_LEVEL_SERVOING)

        # Move arm to ready position
        Logger.log(f"Moving the arm to {name} position")
        action_type = Base_pb2.RequestedActionType()
        action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
        action_list = self.base.ReadAllActions(action_type)
        action_handle = None
        for action_name in action_list.action_list:
            if action_name.name == name:
                action_handle = action_name.handle

        if action_handle is None:
            Logger.log("Can't reach safe position. Exiting")
            return False

        event = Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self._check_for_end_or_abort(event), Base_pb2.NotificationOptions()
        )

        self.base.ExecuteActionFromReference(action_handle)
        finished = event.wait(self.time_out_duration)
        self.base.Unsubscribe(notification_handle)

        if finished:
            Logger.log(f"{name} position reached")
        else:
            Logger.log("Timeout on action notification wait")
            if self.mock:
                Logger.log("Return successful for mock.")
                return True
        return finished

    def _active_controller(self, target: callable, args: list = []) -> None:
        thread = Thread(target=target, args=args)
        thread.start()
        thread.join()
        self.active = False

    def _start_control(self, target: callable, args: list = []) -> None:
        if not self.active:
            self.active = True
            active_controller = Thread(
                target=self._active_controller, args=[target, args]
            )
            active_controller.start()

    def _check_for_end_or_abort(self, event: Event) -> callable:
        """Return a closure checking for END or ABORT notifications."""

        def check(notif: Base_pb2.ActionNotification, event: Event = event) -> None:
            Logger.log("EVENT : " + Base_pb2.ActionEvent.Name(notif.action_event))
            if notif.action_event in [Base_pb2.ACTION_END, Base_pb2.ACTION_ABORT]:
                event.set()

        return check
