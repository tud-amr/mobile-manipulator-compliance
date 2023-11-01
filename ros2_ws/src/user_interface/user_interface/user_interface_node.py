import os
import rclpy
import time
from rclpy.node import Node
from kinova_driver_msg.msg import KinFdbk, KinCmd
from dingo_driver_msg.msg import DinFdbk, DinCmd
from kinova_driver_msg.srv import KinSrv
from simulation_msg.msg import SimFdbk
from simulation_msg.srv import SimSrv
from controller_msg.srv import ConSrv
from compliant_control.interface.user_interface import UserInterface
from compliant_control.interface.templates import Widget, Group
from threading import Thread


class RateCounterNode(Node):
    """A node that counts the update rates."""

    def __init__(self, interface: UserInterface) -> None:
        super().__init__("rate_counter_node")
        self.create_subscription(
            KinFdbk, "/kinova/fdbk", lambda _: interface.rates["kin"].inc(), 10
        )
        self.create_subscription(
            DinFdbk, "/dingo/fdbk", lambda _: interface.rates["din"].inc(), 10
        )
        self.create_subscription(
            KinCmd, "/kinova/cmd", lambda _: interface.rates["con"].inc(), 10
        )
        self.create_subscription(
            SimFdbk, "/sim/fdbk", lambda _: interface.rates["sim"].inc(), 10
        )


class UserInterfaceNode(Node):
    """A node that starts the user interface."""

    def __init__(self, interface: UserInterface) -> None:
        super().__init__("user_interface_node")
        self.create_subscription(KinFdbk, "/kinova/fdbk", self.kin_fdbk, 10)
        self.create_subscription(DinFdbk, "/dingo/fdbk", self.din_fdbk, 10)
        self.kinova_client = self.create_client(KinSrv, "/kinova/srv")
        self.sim_client = self.create_client(SimSrv, "/sim/srv")
        self.dingo_pub = self.create_publisher(DinCmd, "/dingo/cmd", 10)
        self.con_client = self.create_client(ConSrv, "/control/srv")

        self.interface = interface
        self.interface.cb_din = self.command_dingo
        Widget.callback_link = self.callback
        self.interface.create_ui()

    def callback(self, info: list[str]) -> None:
        """Callback."""
        print(info)
        match info[0]:
            case "Kin":
                self.call_kinova(info[1])
            case "Sim":
                self.call_sim(info[1])
            case "Con":
                self.call_con(info[1])
            case None:
                self.call_con(info[1])
                self.call_kinova(info[1])
        Group.update_all()

    def call_UI(self, name: str) -> None:
        """Call a UI method."""
        match name:
            case "Reset wheels":
                self.interface.reset_wheels()

    def call_con(self, name: str) -> None:
        """Call a controller service."""
        print(name)
        request = ConSrv.Request()
        request.name = name
        future = self.con_client.call_async(request)
        while not future.done():
            time.sleep(0.1)
        response: ConSrv.Response = future.result()
        for joint in self.interface.joints:
            joint.ratio = response.joint_ratio[joint.index]
            joint.fric_d = response.joint_fric_d[joint.index]
            joint.fric_s = response.joint_fric_s[joint.index]
        self.interface.state.comp_grav = response.comp_grav
        self.interface.state.comp_fric = response.comp_fric
        self.interface.state.imp_joint = response.imp_joint
        self.interface.state.imp_cart = response.imp_cart

    def call_sim(self) -> None:
        """Call a simulation service."""
        request = SimSrv.Request()
        request.name = "ToggleAutomove"
        self.sim_client.call(request)

    def call_kinova(self, name: str) -> None:
        """Call a kinova service."""
        self.interface.state.mode = "waiting"
        request = KinSrv.Request()
        request.name = name
        future = self.kinova_client.call_async(request)
        while not future.done():
            time.sleep(0.1)
        response: KinSrv.Response = future.result()
        self.interface.state.mode = response.mode
        self.interface.state.servoing = response.servoing_mode
        for joint in self.interface.joints:
            joint.mode = response.control_mode[joint.index][:3]
            joint.active = response.active[joint.index]

    def command_dingo(self, direction: list) -> None:
        """Send a command to Dingo."""
        command = DinCmd()
        command.direction = direction
        self.dingo_pub.publish(command)

    def kin_fdbk(self, msg: KinFdbk) -> None:
        """Update the Kinova feedback."""
        for n in range(len(msg.joint_pos)):
            joint = self.interface.joints[n]
            joint.pos = msg.joint_pos[joint.index]
            joint.vel = msg.joint_vel[joint.index]
            joint.eff = msg.joint_tor[joint.index]
        self.interface.state.update_rate = msg.update_rate
        self.interface.update_bars("Kinova")

    def din_fdbk(self, msg: DinFdbk) -> None:
        """Update the Dingo feedback."""
        for n in range(len(msg.wheel_pos)):
            wheel = self.interface.wheels[n]
            last_position = wheel.pos
            wheel.encoder_position = msg.wheel_pos[n]
            wheel.vel = wheel.pos - last_position
            wheel.eff = msg.wheel_tor[n]
        self.interface.update_bars("Dingo")


def main(args: any = None) -> None:
    """Main."""
    rclpy.init(args=args)
    executor: rclpy.Executor = rclpy.executors.MultiThreadedExecutor()
    interface = UserInterface()
    executor.add_node(RateCounterNode(interface))
    executor.add_node(UserInterfaceNode(interface))
    spin_thread = Thread(target=executor.spin)
    spin_thread.start()
    interface.start_render_loop()
    rclpy.shutdown()
    os._exit(0)


if __name__ == "__main__":
    main()
