import time
from threading import Thread, Event
from typing import Literal
import numpy as np

from kortex_api.RouterClient import RouterClient
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.client_stubs.ActuatorConfigClientRpc import ActuatorConfigClient
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, ActuatorConfig_pb2
from kortex_api.Exceptions.KServerException import KServerException

from .specifications import Position, actuator_ids, ranges

from compliant_control.control.state import State


class KortexClient:
    """Class that uses the Kortex API to communicate with the robot."""

    def __init__(
        self,
        state: State,
        base: BaseClient = None,
        base_cyclic: BaseCyclicClient = None,
        actuator_config: ActuatorConfigClient = None,
        router: RouterClient = None,
        real_time_router: RouterClient = None,
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

        self.log = lambda msg: print(msg)

        self.state = state

        self.simulate = simulate
        self.actuator_count = self.base.GetActuatorCount().count
        self.joint_active = [n % 2 == 0 for n in range(self.actuator_count)]

        self.calibrating = False
        self.changing_servoing_mode = False
        self.controller_connected = False
        self.active = False

        self.frequency = 1000
        self.rate = self.frequency
        self.n = self.frequency
        self.sleep_time = 1 / self.frequency

        self.mode = "HLC"

        self._set_servoing_mode(Base_pb2.SINGLE_LEVEL_SERVOING)
        self._refresh()
        self._initialize_command()
        self.start()

    def get_mode(self) -> str:
        """Get the general mode."""
        return "calibrating" if self.calibrating else self.mode

    def get_control_modes(self) -> list[str]:
        """Get the control mode of an actuator."""
        return [
            ActuatorConfig_pb2.ControlMode.Name(self.actuator_modes[n])
            for n in range(self.actuator_count)
        ]

    def get_servoing_mode(self) -> str:
        """Get the servoing mode of the robot."""
        return Base_pb2.ServoingMode.Name(self.servoing_mode)

    def get_update_rate(self) -> int:
        """Get the update rate."""
        return self.rate

    def clear_faults(self) -> None:
        """Clear the faults."""
        self.base.ClearFaults()

    def start(self) -> None:
        """Start the refresh loop."""
        rate_check_thread = Thread(target=self._rate_check_loop)
        refresh_loop_thread = Thread(target=self._refresh_loop)
        self.active = True
        rate_check_thread.start()
        refresh_loop_thread.start()

    def stop(self, *args: any) -> None:
        """Stop the update loop."""
        print("Closing connection with arm...")
        self.active = False

    def set_control_mode(
        self, joint: int, mode: Literal["position", "velocity", "current"]
    ) -> None:
        """Set the control mode of an actuator."""
        mode = getattr(ActuatorConfig_pb2, mode.upper())
        control_mode_information = ActuatorConfig_pb2.ControlModeInformation()
        control_mode_information.control_mode = mode
        _id = joint if self.simulate else actuator_ids[joint]
        self.actuator_config.SetControlMode(control_mode_information, _id)
        self._update_modes()

    def start_LLC(self) -> None:
        """Start low_level control."""
        self.copy_feedback_to_command()
        for n in range(self.actuator_count):
            self.set_control_mode(n, "position")
        self._set_servoing_mode(Base_pb2.LOW_LEVEL_SERVOING)
        self.mode = "LLC"
        self.log("Low_level control enabled.")

    def stop_LLC(self) -> None:
        """Stop low_level control."""
        self._set_servoing_mode(Base_pb2.SINGLE_LEVEL_SERVOING)
        self.mode = "HLC"
        self.log("Low_level control disabled.")

    def connect_LLC(self) -> None:
        """Connect a controller to the LLC of the robot."""
        self.copy_feedback_to_command()
        for n in range(self.actuator_count):
            if self.joint_active[n]:
                self.base_cyclic.Refresh(self.command)
                self.set_control_mode(n, "current")
        self.controller_connected = True
        self.mode = "LLC_task"
        self.log("Controller connected.")

    def disconnect_LLC(self) -> None:
        """Disconnect a controller from the LLC of the robot."""
        self.controller_connected = False
        for joint in range(self.actuator_count):
            self.set_control_mode(joint, "position")
        self.mode = "LLC"
        self.log("Controller disconnected.")

    def home(self) -> bool:
        """Move the arm to the home position."""
        self._high_level_move(Position.home)

    def zero(self) -> bool:
        """Move the arm to the zero position."""
        self._high_level_move(Position.zero)

    def retract(self) -> bool:
        """Move the arm to the retract position."""
        self._high_level_move(Position.retract)

    def pref(self) -> bool:
        """Move the arm to the pref position."""
        self._high_level_move(Position.pref)

    def get_position(self, joint: int, as_percentage: bool) -> float:
        """Get the position of a joint."""
        position = getattr(self.feedback.actuators[joint], "position")
        if self.simulate:
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
        if self.simulate:
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

    def copy_feedback_to_command(self) -> None:
        """Copy the feedback to the command message."""
        for prop in ["position", "velocity", "current_motor"]:
            for n in range(self.actuator_count):
                value = getattr(self.feedback.actuators[n], prop)
                setattr(self.command.actuators[n], prop, value)

    def set_command(self, commands: list) -> None:
        """Set the command."""
        self.copy_feedback_to_command()
        for n, command in enumerate(commands):
            if self.joint_active[n]:
                self.command.actuators[n].current_motor = command

    def toggle_active(self, joint: int) -> None:
        """Toggle active state of joint."""
        self.joint_active[joint] = not self.joint_active[joint]

    def update_state(self) -> None:
        """Update the state."""
        for n in range(self.actuator_count):
            self.state.kinova_feedback.q[n] = self.get_position(n, False)
            self.state.kinova_feedback.dq[n] = self.get_velocity(n, False)
            self.state.kinova_feedback.c[n] = self.get_current(n, False)

    def _refresh_loop(self) -> bool:
        while self.active:
            self._refresh()
            self.update_state()
            if self.mode == "LLC_task":
                self.state.controller.command()
                self.set_command(self.state.controller.joint_commands)
            self.n += 1
            if self.simulate:
                time.sleep(self.sleep_time)

    def _refresh(self) -> None:
        """Refresh."""
        if not self.changing_servoing_mode and self.controller_connected:
            try:
                self.feedback = self.base_cyclic.Refresh(self.command)
            except KServerException:
                self.log("Robot control lost.")
                self.controller_connected = False
        else:
            self.feedback = self.base_cyclic.RefreshFeedback()

    def _rate_check_loop(self) -> None:
        """Define te rate check loop."""
        while self.active:
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
            _id = n if self.simulate else actuator_ids[n]
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

    def _high_level_move(self, position: Position) -> None:
        """Perform a high level move."""
        self.log("Starting high level movement...")
        action = Base_pb2.Action()
        action.name = position.name
        action.application_data = ""

        for n, pos in enumerate(position.position):
            joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
            joint_angle.joint_identifier = n
            joint_angle.value = pos

        return self._execute_action(action)

    def _execute_action(self, action: Base_pb2.Action = None) -> bool:
        event = Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self._check_for_end_or_abort(event), Base_pb2.NotificationOptions()
        )

        self.base.ExecuteAction(action)
        finished = event.wait(3)
        self.base.Unsubscribe(notification_handle)

        if finished:
            self.log(f"Position {action.name} reached")
        else:
            self.log("Timeout on action notification wait")
        return finished

    def _check_for_end_or_abort(self, event: Event) -> callable:
        """Return a closure checking for END or ABORT notifications."""

        def check(notif: Base_pb2.ActionNotification, event: Event = event) -> None:
            self.log("EVENT : " + Base_pb2.ActionEvent.Name(notif.action_event))
            if notif.action_event in [Base_pb2.ACTION_END, Base_pb2.ACTION_ABORT]:
                event.set()

        return check
