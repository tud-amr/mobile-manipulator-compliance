import time
import numpy as np
from kortex_api.autogen.messages import ActuatorConfig_pb2
from compliant_control.kinova.kortex_client import KortexClient
from compliant_control.kinova.messages import Base, BaseCyclic, ActuatorConfig
from compliant_control.kinova.specifications import Position

from rclpy.node import Node
from simulation_msg.msg import SimFdbk, SimCmd, SimCmdInc
from simulation_msg.srv import SimSrv

JOINTS = 6


class KortexClientSimulation(KortexClient):
    """A mock of the Kortex Client class, to make testing without robot possible."""

    def __init__(self, node: Node) -> None:
        self.base = BaseClientSimulation()
        self.base_cyclic = BaseCyclicClientSimulation(node)
        self.actuator_config = ActuatorConfigClientSimulation(node)
        super().__init__(
            base=self.base,
            base_cyclic=self.base_cyclic,
            actuator_config=self.actuator_config,
            mock=True,
            simulate=True,
        )
        node.create_subscription(SimFdbk, "/sim/fdbk", self.sim_fdbk, 10)
        self.pub = node.create_publisher(SimCmdInc, "/sim/cmd_inc", 10)
        self.define_HLC_parameters()

    def sim_fdbk(self, msg: SimFdbk) -> None:
        """Process the feedback from the simulation."""
        for n in range(JOINTS):
            self.base_cyclic.feedback.actuators[n].position = msg.joint_pos[n]
            self.base_cyclic.feedback.actuators[n].velocity = msg.joint_vel[n]
            self.base_cyclic.feedback.actuators[n].current_motor = msg.joint_tor[n]

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
        msg = SimCmdInc()
        msg.joint = joint
        msg.type = "position"
        if error > 0:
            msg.increment = step
        elif error < 0:
            msg.increment = -step
        self.pub.publish(msg)


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

    def __init__(self, node: Node) -> None:
        self.pub = node.create_publisher(SimCmd, "/sim/cmd", 10)
        self.feedback = BaseCyclic.Feedback(JOINTS)

    def Refresh(self, command: BaseCyclic.Command) -> BaseCyclic.Feedback:
        """Send a command to actuators and interface and returns feedback from base, actuators, and interface on actual status."""
        command_msg = SimCmd()
        command_msg.robot = "Kinova"
        for actuator in command.actuators:
            command_msg.joint_pos.append(actuator.position)
            command_msg.joint_vel.append(actuator.velocity)
            command_msg.joint_tor.append(actuator.current_motor)
        self.pub.publish(command_msg)
        return self.RefreshFeedback()

    def RefreshFeedback(self) -> BaseCyclic.Feedback:
        """Obtain feedback from base, actuators, and interface on their status."""
        return self.feedback


class ActuatorConfigClientSimulation:
    """https://github.com/Kinovarobotics/kortex/blob/master/api_python/doc/markdown/summary_pages/ActuatorConfig.md."""

    def __init__(self, node: Node) -> None:
        self.client = node.create_client(SimSrv, "/sim/srv")
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
        request = SimSrv.Request()
        request.arg = device_id
        if mode_value == ActuatorConfig_pb2.POSITION:
            request.name = "KinovaPositionMode"
        elif mode_value == ActuatorConfig_pb2.CURRENT:
            request.name = "KinovaTorqueMode"
        future = self.client.call_async(request)
        while not future.done():
            time.sleep(0.1)
        if future.result().success:
            self.control_modes[device_id].control_mode = mode_value
