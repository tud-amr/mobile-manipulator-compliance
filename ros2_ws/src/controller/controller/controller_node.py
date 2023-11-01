import rclpy
import os
import signal
from rclpy.node import Node
import numpy as np

from kinova_driver_msg.msg import KinFdbk, KinCmd, KinTar
from dingo_driver_msg.msg import DinFdbk, DinCmd
from controller_msg.srv import ConSrv
from simulation_msg.srv import SimSrv

from compliant_control.control.state import State
from compliant_control.control.controller import Controller


class TargetClient(Node):
    """Target client to reset the target."""

    def __init__(self) -> None:
        super().__init__("controller_target_client")
        self.sim_client = self.create_client(SimSrv, "/sim/srv")

    def reset_target(self) -> None:
        """Reset the target."""
        request = SimSrv.Request()
        request.name = "ResetTarget"
        self.sim_client.call(request)


class ControllerServiceNode(Node):
    """Service node for the controller."""

    def __init__(
        self, state: State, controller: Controller, target_client: TargetClient
    ) -> None:
        super().__init__("controller_service_node")
        self.create_service(ConSrv, "/control/srv", self.service_call)

        self.state = state
        self.controller = controller
        self.target_client = target_client

    def service_call(
        self, request: ConSrv.Request, response: ConSrv.Response
    ) -> ConSrv.Response:
        """Execute service call."""
        print(f"CON CALL: {request.name}")
        match request.name:
            case "Refresh":
                pass
            case "Start LLC Task":
                self.target_client.reset_target()
                self.controller.reset()
                self.controller.active = True
            case "Stop LLC Task":
                self.controller.active = False
            case _ if request.name in ["gravity", "friction", "joint", "cartesian"]:
                if request.name in ["joint", "cartesian"]:
                    self.target_client.reset_target()
                self.controller.toggle(request.name)
            case _:
                print(f"Service call {request.name} is unknown.")

        response.joint_ratio = list(self.state.ratios)
        response.joint_fric_s = self.state.static_frictions
        response.joint_fric_d = self.state.dynamic_frictions
        response.comp_grav = self.controller.comp_grav
        response.comp_fric = self.controller.comp_fric
        response.imp_joint = self.controller.imp_joint
        response.imp_cart = self.controller.imp_cart
        return response


class ControllerNode(Node):
    """Main controller that can control the kinova arm and dingo base."""

    def __init__(self, state: State, controller: Controller) -> None:
        super().__init__("controller_node")
        self.create_subscription(KinFdbk, "/kinova/fdbk", self.kinova_fdbk, 10)
        self.create_subscription(KinTar, "/kinova/tar", self.kinova_tar, 10)
        self.create_subscription(DinFdbk, "/dingo/fdbk", self.dingo_fdbk, 10)
        self.kinova_cmd = self.create_publisher(KinCmd, "/kinova/cmd", 10)
        self.dingo_cmd = self.create_publisher(DinCmd, "/dingo/cmd", 10)

        self.state = state
        self.controller = controller
        self.controller.reset()

    def command(self) -> None:
        """Send a command."""
        self.controller.command()
        cmd = KinCmd()
        cmd.joint_command = list(self.controller.joint_commands)
        self.kinova_cmd.publish(cmd)
        cmd = DinCmd()
        cmd.direction = list(self.controller.base_command)
        self.dingo_cmd.publish(cmd)

    def kinova_fdbk(self, msg: KinFdbk) -> None:
        """Process the feedback from the kinova arm."""
        self.state.q = np.array(msg.joint_pos)
        self.state.dq = np.array(msg.joint_vel)
        if self.controller.active:
            self.command()

    def kinova_tar(self, msg: KinTar) -> None:
        """Process the target."""
        self.state.target = np.array(msg.target)

    def dingo_fdbk(self, msg: KinFdbk) -> None:
        """Process the feedback from the dingo base."""


def main(args: any = None) -> None:
    """Main."""
    rclpy.init(args=args)
    executor: rclpy.Executor = rclpy.executors.MultiThreadedExecutor()
    state = State()
    controller = Controller(state)
    executor.add_node(ControllerNode(state, controller))
    target_client = TargetClient()
    executor.add_node(target_client)
    executor.add_node(ControllerServiceNode(state, controller, target_client))

    signal.signal(signal.SIGINT, lambda *_: executor.shutdown(0))
    while not executor._is_shutdown:
        executor.spin_once(timeout_sec=0)
    os._exit(0)


if __name__ == "__main__":
    main()
