from __future__ import annotations
from threading import Thread, Event
from typing import Literal
import numpy as np

from kortex_api.RouterClient import RouterClient
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient, BaseFunctionUid
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.client_stubs.ActuatorConfigClientRpc import ActuatorConfigClient
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, ActuatorConfig_pb2
from kortex_api.Exceptions.KServerException import KServerException

from .specifications import Position, actuator_ids, ranges

from compliant_control.control.state import State
from compliant_control.utilities.rate_counter import RateCounter


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
        self.router = router
        self.real_time_router = router

        self.simulate = simulate
        self.actuator_count = self.base.GetActuatorCount().count
        self.joint_active = [True, True, True, True, True, True]

        self.calibrating = False
        self.changing_servoing_mode = False
        self.controller_connected = False
        self.active = False

        self.rate_counter = RateCounter(1200)

        self.mode = "HLC"

        self._set_servoing_mode(Base_pb2.SINGLE_LEVEL_SERVOING)
        self._refresh()
        self._initialize_command()
        self._desired_gripper_position = 50

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

    def clear_faults(self) -> None:
        """Clear the faults."""
        self.base.ClearFaults()

    def start_in_new_thread(self) -> None:
        """Start the refresh loop in a new thread."""
        thread = Thread(target=self.start)
        thread.start()

    def start(self) -> None:
        """Start the refresh loop."""
        self.active = True
        self._refresh_loop()

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

    def start_HLT(self) -> None:
        """Start high_level tracking."""
        thread = Thread(target=self._high_level_tracking)
        thread.start()

    def stop_HLT(self) -> None:
        """Stop high_level tracking."""
        self.mode = "HLC"

    def start_LLC(self) -> None:
        """Start low_level control."""
        self.copy_feedback_to_command()
        current_gripper_position = self.feedback.interconnect.gripper_feedback.motor[0].position
        self.gripper_command.position = current_gripper_position
        self._desired_gripper_position = current_gripper_position
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
        self.state.controller.start_control_loop()
        for n in range(self.actuator_count):
            if self.joint_active[n]:
                self.copy_feedback_to_command()
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
        self.state.controller.stop_control_loop()
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

    def high_level_move(self, position: Position) -> None:
        """Perform a high level movement."""
        self.mode = "high_level_moving"
        self._high_level_move(position)
        self.mode = "HLC"

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

    def get_gripper_position(self) -> float:
        """Get the gripper position."""
        return (self.feedback.interconnect.gripper_feedback.motor[0].position) / 100

    def get_torque(self, joint: int, as_percentage: bool) -> float:
        """Get the torque of a joint."""
        torque = getattr(self.feedback.actuators[joint], "torque")
        if as_percentage:
            lower_bound = ranges["torque"][joint][0]
            upper_bound = ranges["torque"][joint][1]
            return (abs(torque) - lower_bound) / (upper_bound - lower_bound)
        return getattr(self.feedback.actuators[joint], "torque")

    def copy_feedback_to_command(self, joint: int = None) -> None:
        """Copy the feedback to the command message."""
        for prop in ["position", "velocity", "current_motor"]:
            for n in range(self.actuator_count) if joint is None else [joint]:
                value = getattr(self.feedback.actuators[n], prop)
                setattr(self.command.actuators[n], prop, value)

    def set_command(self, commands: list) -> None:
        """Set the command."""
        for n, command in enumerate(commands):
            if self.joint_active[n]:
                self.copy_feedback_to_command(n)
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
            self.state.kinova_feedback.fault[n] = self.feedback.actuators[n].fault_bank_a

    def _refresh_loop(self) -> bool:
        while self.active:

            self._refresh()
            self.update_state()
            if self.mode == "LLC_task":
                current_gripper_position = self.feedback.interconnect.gripper_feedback.motor[0].position
                position_error = self._desired_gripper_position - current_gripper_position
                clipped_position_error = np.clip(position_error, -5.0, 5.0)
                set_gripper_position = current_gripper_position + clipped_position_error
                self.gripper_command.position = current_gripper_position + clipped_position_error
                if not np.any(self.state.kinova_feedback.fault):
                    self.set_command(self.state.controller.joint_commands)
                else:
                    self.state.controller.stop_control_loop()
            self.rate_counter.count()
            if self.simulate:
                self.rate_counter.sleep()

    def _refresh(self) -> None:
        """Refresh."""
        if not self.changing_servoing_mode and self.controller_connected:
            try:
                self.feedback = self.base_cyclic.Refresh(self.command)
            except Exception as e:
                self.log(f"Exception: {e}")
                self.controller_connected = False
        else:
            self.feedback = self.base_cyclic.RefreshFeedback()

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
        self.gripper_command = self.command.interconnect.gripper_command.motor_cmd.add()
        self.command.interconnect.command_id.identifier = 0
        self.command.interconnect.gripper_command.command_id.identifier = 6
        self.gripper_command.velocity = 0.0
        self.gripper_command.force = 5

    def _high_level_move(self, position: Position) -> None:
        """Perform a high level move."""
        action = Base_pb2.Action()
        action.name = position.name
        action.application_data = ""

        for n, pos in enumerate(position.position):
            joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
            joint_angle.joint_identifier = n
            joint_angle.value = pos

        return self._execute_action(action)

    def _move_gripper(self, close: bool) -> None:
        if self.mode == 'HLC':
          gripper_command = Base_pb2.GripperCommand()
          gripper_command.mode = Base_pb2.GRIPPER_POSITION
          finger = gripper_command.gripper.finger.add()
          finger.finger_identifier = 1
          pos = self.get_gripper_position()
          if close:
              finger.value = min(1, pos + 0.1)
          else:
              finger.value = max(0, pos - 0.1)

          self.base.SendGripperCommand(gripper_command)
        elif self.mode == 'LLC_task':
          if close:
            self.move_gripper_to_position(90)
          else:
            self.move_gripper_to_position(10)

    def move_gripper_to_position(self, position: float):
        self._desired_gripper_position = position
        

    def _high_level_tracking(self) -> None:
        self.mode = "HLT"
        joint_speeds = Base_pb2.JointSpeeds()
        gain = 10
        lower_threshold = 0.001
        upper_threshold = 0.05
        q_pref = np.deg2rad(Position.pref.position)
        n_gain = 150
        n_max = 30
        while self.mode == "HLT":
            dq = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            error = self.state.target - self.state.x
            m = np.linalg.norm(error)
            if m > lower_threshold:
                dx = error / m
                dx *= min((m / upper_threshold), 1) * gain
                dq = self.state.dq_inv(dx)
            dq_n = q_pref - self.state.kinova_feedback.q
            dq_n = np.sign(dq_n) * np.minimum(n_max, n_gain * np.absolute(dq_n))
            dq_n = self.state.Nv @ dq_n
            dq += dq_n
            del joint_speeds.joint_speeds[:]
            for n, speed in enumerate(dq):
                joint_speed = joint_speeds.joint_speeds.add()
                joint_speed.joint_identifier = n
                joint_speed.value = speed
                joint_speed.duration = 0

            self.base.SendJointSpeedsCommand(joint_speeds)
        self.base.Stop()

    def _execute_action(self, action: Base_pb2.Action = None) -> bool:
        event = Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self._check_for_end_or_abort(event), Base_pb2.NotificationOptions()
        )

        self.base.ExecuteAction(action)
        finished = event.wait(30)
        self.base.Unsubscribe(notification_handle)

        if finished:
            self.log(f"Position {action.name} reached")
        else:
            self.log("Timeout on action notification wait")
        return finished

    def _check_for_end_or_abort(self, event: Event) -> callable:
        """Return a closure checking for END or ABORT notifications."""

        def check(notif: Base_pb2.ActionNotification, event: Event = event) -> None:
            if notif.action_event in [Base_pb2.ACTION_END, Base_pb2.ACTION_ABORT]:
                event.set()

        return check
