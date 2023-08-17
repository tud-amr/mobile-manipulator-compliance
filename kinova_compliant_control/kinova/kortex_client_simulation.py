import mujoco
import numpy as np
from kinova.kortex_client import KortexClient
from user_interface.mujoco_viewer import MujocoViewer
from kortex_api.autogen.messages import ActuatorConfig_pb2
from compliant_controller.state import State
from kinova.messages import Base, BaseCyclic, ActuatorConfig
from kinova.specifications import Position
import time


class KortexClientSimulation(KortexClient):
    """A mock of the Kortex Client class, to make testing without robot possible."""

    def __init__(self, state: State, mujoco_viewer: MujocoViewer) -> None:
        super().__init__(
            state,
            base=BaseClientSimulation(mujoco_viewer),
            base_cyclic=BaseCyclicClientSimulation(mujoco_viewer),
            actuator_config=ActuatorConfigClientSimulation(mujoco_viewer),
            mock=True,
            simulate=True,
        )
        self.model = mujoco_viewer.model
        self.data = mujoco_viewer.data
        for n in range(self.actuator_count):
            self.data.qpos[n] = self.data.ctrl[n] = np.deg2rad(
                Position.home.position[n]
            )

        self.define_HLC_parameters()

    def define_HLC_parameters(self) -> None:
        """Define the HLC parameters."""
        self.reached_error = 10**-5  # deg
        self.nearby_goal_divider = 100

        moving_speed = 20  # deg/s
        step_deg = moving_speed / self.frequency
        self.step_size = np.deg2rad(step_deg)

    def _high_level_move(self, position: Position) -> None:
        pose = np.deg2rad(position.position)
        reached = [False] * self.actuator_count
        while not all(reached):
            for joint in [n for n, done in enumerate(reached) if not done]:
                error = abs(pose[joint] - self.state.q[joint])
                if error < self.reached_error:
                    reached[joint] = True
                else:
                    self._execute_action(joint, pose, error)
            time.sleep(1 / self.frequency)

    def _execute_action(
        self, joint: int, pose: list[float], error: list[float]
    ) -> bool:
        step = min(error / self.nearby_goal_divider, self.step_size)
        if self.state.q[joint] > pose[joint]:
            self.data.ctrl[joint] -= step
        elif self.state.q[joint] < pose[joint]:
            self.data.ctrl[joint] += step


class BaseClientSimulation:
    """https://github.com/Kinovarobotics/kortex/blob/master/api_python/doc/markdown/summary_pages/Base.md."""

    def __init__(self, mujoco_viewer: MujocoViewer) -> None:
        self.model = mujoco_viewer.model
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
        return Base.ActuatorInformation(self.model.njnt)


class BaseCyclicClientSimulation:
    """https://github.com/Kinovarobotics/kortex/blob/master/api_python/doc/markdown/summary_pages/BaseCyclic.md."""

    def __init__(self, mujoco_viewer: MujocoViewer) -> None:
        self.model = mujoco_viewer.model
        self.data = mujoco_viewer.data
        self.feedback = BaseCyclic.Feedback(self.model.njnt)

    def Refresh(self, command: BaseCyclic.Command) -> BaseCyclic.Feedback:
        """Send a command to actuators and interface and returns feedback from base, actuators, and interface on actual status."""
        for n, actuator in enumerate(command.actuators):
            self.data.ctrl[n] = actuator.position
            n += self.model.njnt
            self.data.ctrl[n] = actuator.velocity
            n += self.model.njnt
            self.data.ctrl[n] = actuator.current_motor
        return self.RefreshFeedback()

    def RefreshFeedback(self) -> BaseCyclic.Feedback:
        """Obtain feedback from base, actuators, and interface on their status."""
        for n in range(self.model.nsensor):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_SENSOR, n)
            joint = int(name[-1])
            attr = name[:-1]
            setattr(self.feedback.actuators[joint], attr, self.data.sensordata[n])
        for n in range(self.model.njnt):
            torque = 0
            m = n
            for _ in range(3):
                torque += self.data.actuator_force[m]
                m += self.model.njnt
            setattr(self.feedback.actuators[n], "torque", torque)
        return self.feedback


class ActuatorConfigClientSimulation:
    """https://github.com/Kinovarobotics/kortex/blob/master/api_python/doc/markdown/summary_pages/ActuatorConfig.md."""

    def __init__(self, mujoco_viewer: MujocoViewer) -> None:
        self.model = mujoco_viewer.model
        self.default_biasprm = self.model.actuator_biasprm.copy()
        self.default_gainprm = self.model.actuator_gainprm.copy()
        self.control_modes = [
            ActuatorConfig.ControlModeInformation(1) for _ in range(self.model.njnt)
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
        joint = device_id
        if mode_value == ActuatorConfig_pb2.POSITION:
            self.model.actuator_biasprm[joint][1] = self.default_biasprm[joint][1]
            self.model.actuator_gainprm[joint][0] = self.default_gainprm[joint][0]
            joint += self.model.njnt
            self.model.actuator_biasprm[joint][2] = self.default_biasprm[joint][2]
            self.model.actuator_gainprm[joint][0] = self.default_gainprm[joint][0]
        elif mode_value == ActuatorConfig_pb2.CURRENT:
            self.model.actuator_biasprm[joint][1] = 0
            self.model.actuator_gainprm[joint][0] = 0
            joint += self.model.njnt
            self.model.actuator_biasprm[joint][2] = 0
            self.model.actuator_gainprm[joint][0] = 0
        self.control_modes[device_id].control_mode = mode_value
