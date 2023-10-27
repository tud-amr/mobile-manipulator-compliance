import rclpy
import os
from rclpy.node import Node
import numpy as np

from kinova_driver_msg.msg import KinFdbk, KinCmd
from dingo_driver_msg.msg import DinFdbk
from controller_msg.srv import ConSrv

from compliant_control.controllers.state import State
from compliant_control.controllers.controllers import Controllers, Controller


class ControllerServiceNode(Node):
    """Service node for the controller."""

    def __init__(self, state: State) -> None:
        super().__init__("controller_service_node")
        self.create_service(ConSrv, "/control/srv", self.service_call)

        self.state = state

    def service_call(
        self, request: ConSrv.Request, response: ConSrv.Response
    ) -> ConSrv.Response:
        """Execute service call."""
        match request.name:
            case "Compensate friction":
                Controller.toggle_CF()
            case _:
                print(f"Service call {request.name} is unknown.")

        response.joint_ratio = list(self.state.ratios)
        response.joint_fric_s = self.state.static_frictions
        response.joint_fric_d = self.state.dynamic_frictions
        response.compensate_friction = Controller.get_CF()
        return response


class ControllerNode(Node):
    """Main controller that can control the kinova arm and dingo base."""

    def __init__(self, state: State) -> None:
        super().__init__("controller_node")
        self.create_subscription(KinFdbk, "/kinova/fdbk", self.kinova_fdbk, 10)
        self.create_subscription(DinFdbk, "/dingo/fdbk", self.dingo_fdbk, 10)
        self.kinova_cmd = self.create_publisher(KinCmd, "/kinova/cmd", 10)

        self.state = state
        self.state.simulation = True
        self.controllers = Controllers(self.state)
        self.controller = self.controllers.compensate_gravity_and_friction
        self.controller.reset_before_connect()

    def command(self) -> None:
        """Send a command."""
        self.controller.command()
        cmd = KinCmd()
        cmd.joint_command = list(self.controller.commands)
        self.kinova_cmd.publish(cmd)

    def kinova_fdbk(self, msg: KinFdbk) -> None:
        """Process the feedback from the kinova arm."""
        self.state.q = np.array(msg.joint_pos)
        self.state.dq = np.array(msg.joint_vel)
        self.state.update()
        self.command()

    def dingo_fdbk(self, msg: KinFdbk) -> None:
        """Process the feedback from the dingo base."""


def main(args: any = None) -> None:
    """Main."""
    rclpy.init(args=args)
    executor: rclpy.Executor = rclpy.executors.MultiThreadedExecutor()
    state = State()
    executor.add_node(ControllerNode(state))
    executor.add_node(ControllerServiceNode(state))
    executor.spin()
    rclpy.shutdown()
    os._exit(0)


if __name__ == "__main__":
    main()
