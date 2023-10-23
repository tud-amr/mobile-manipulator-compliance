import time
from mujoco import mj_name2id, mjtObj
import numpy as np
from kortex_api.autogen.messages import ActuatorConfig_pb2
from compliant_control.kinova.kortex_client import KortexClient
from compliant_control.kinova.mujoco_viewer import MujocoViewer
from compliant_control.kinova.messages import Base, BaseCyclic, ActuatorConfig
from compliant_control.kinova.specifications import Position

JOINTS = 6


class KortexClientSimulation(KortexClient):
    """A mock of the Kortex Client class, to make testing without robot possible."""

    def __init__(self, mujoco_viewer: MujocoViewer) -> None:
        self.mujoco_viewer = mujoco_viewer
        super().__init__(
            base=BaseClientSimulation(self.mujoco_viewer),
            base_cyclic=BaseCyclicClientSimulation(self.mujoco_viewer),
            actuator_config=ActuatorConfigClientSimulation(self.mujoco_viewer),
            mock=True,
            simulate=True,
        )
        self.model = self.mujoco_viewer.model
        self.data = self.mujoco_viewer.data
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
                position = self.get_position(joint, False)
                error = pose[joint] - position
                if abs(error) < self.reached_error:
                    reached[joint] = True
                else:
                    self._execute_action(joint, error)
            time.sleep(1 / self.frequency)

    def _execute_action(self, joint: int, error: float) -> bool:
        step = min(abs(error) / self.nearby_goal_divider, self.step_size)
        if error > 0:
            self.data.ctrl[joint] += step
        elif error < 0:
            self.data.ctrl[joint] -= step


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
        return Base.ActuatorInformation(JOINTS)


class BaseCyclicClientSimulation:
    """https://github.com/Kinovarobotics/kortex/blob/master/api_python/doc/markdown/summary_pages/BaseCyclic.md."""

    def __init__(self, mujoco_viewer: MujocoViewer) -> None:
        self.model = mujoco_viewer.model
        self.data = mujoco_viewer.data
        self.feedback = BaseCyclic.Feedback(JOINTS)

    def Refresh(self, command: BaseCyclic.Command) -> BaseCyclic.Feedback:
        """Send a command to actuators and interface and returns feedback from base, actuators, and interface on actual status."""
        for n, actuator in enumerate(command.actuators):
            for x in ["P", "V", "M"]:
                idx = mj_name2id(self.model, mjtObj.mjOBJ_ACTUATOR, f"K_{x}A{n}")
                match x:
                    case "P":
                        self.data.ctrl[idx] = actuator.position
                    case "V":
                        self.data.ctrl[idx] = actuator.velocity
                    case "M":
                        self.data.ctrl[idx] = actuator.current_motor
        return self.RefreshFeedback()

    def RefreshFeedback(self) -> BaseCyclic.Feedback:
        """Obtain feedback from base, actuators, and interface on their status."""
        for n in range(JOINTS):
            idpos = mj_name2id(self.model, mjtObj.mjOBJ_SENSOR, f"K_PS{n}")
            idvel = mj_name2id(self.model, mjtObj.mjOBJ_SENSOR, f"K_VS{n}")
            setattr(self.feedback.actuators[n], "position", self.data.sensordata[idpos])
            setattr(self.feedback.actuators[n], "velocity", self.data.sensordata[idvel])
            torque = 0
            for x in ["P", "V", "M"]:
                idx = mj_name2id(self.model, mjtObj.mjOBJ_ACTUATOR, f"K_{x}A{n}")
                torque += self.data.actuator_force[idx]
            setattr(self.feedback.actuators[n], "torque", torque)
        return self.feedback


class ActuatorConfigClientSimulation:
    """https://github.com/Kinovarobotics/kortex/blob/master/api_python/doc/markdown/summary_pages/ActuatorConfig.md."""

    def __init__(self, mujoco_viewer: MujocoViewer) -> None:
        self.model = mujoco_viewer.model
        self.default_biasprm = self.model.actuator_biasprm.copy()
        self.default_gainprm = self.model.actuator_gainprm.copy()
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
        for x in ["P", "V"]:
            idx = mj_name2id(self.model, mjtObj.mjOBJ_ACTUATOR, f"K_{x}A{device_id}")
            if mode_value == ActuatorConfig_pb2.POSITION:
                if x == "P":
                    self.model.actuator_biasprm[idx][1] = self.default_biasprm[idx][1]
                    self.model.actuator_gainprm[idx][0] = self.default_gainprm[idx][0]
                elif x == "V":
                    self.model.actuator_biasprm[idx][2] = self.default_biasprm[idx][2]
                    self.model.actuator_gainprm[idx][0] = self.default_gainprm[idx][0]
            elif mode_value == ActuatorConfig_pb2.CURRENT:
                if x == "P":
                    self.model.actuator_biasprm[idx][1] = 0
                    self.model.actuator_gainprm[idx][0] = 0
                elif x == "V":
                    self.model.actuator_biasprm[idx][2] = 0
                    self.model.actuator_gainprm[idx][0] = 0
        self.control_modes[device_id].control_mode = mode_value
