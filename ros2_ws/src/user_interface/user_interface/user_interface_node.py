import os
import rclpy
from rclpy.node import Node
from kinova_driver_msg.msg import KinFdbk, KinSts
from dingo_driver_msg.msg import DinFdbk, DinCmd
from kinova_driver_msg.srv import KinSrv
from simulation_msg.srv import SimSrv
from compliant_control.interface.user_interface import UserInterface
from threading import Thread


class UserInterfaceNode(Node):
    """A node that starts the user interface."""

    def __init__(self) -> None:
        super().__init__("user_interface_node")
        self.create_subscription(KinFdbk, "/kinova/fdbk", self.kin_fdbk, 10)
        self.create_subscription(DinFdbk, "/dingo/fdbk", self.din_fdbk, 10)
        self.create_subscription(KinSts, "/kinova/sts", self.kin_sts, 10)
        self.kinova_client = self.create_client(KinSrv, "/kinova/srv")
        self.sim_client = self.create_client(SimSrv, "/sim/srv")
        self.dingo_pub = self.create_publisher(DinCmd, "/dingo/cmd", 10)

        self.interface = UserInterface()
        self.interface.cb_kin = self.call_kinova
        self.interface.cb_din = self.command_dingo
        self.interface.cb_sim = self.call_sim
        self.interface.create_ui()

        spin_thread = Thread(target=self.start_spin_loop)
        spin_thread.start()
        initialize_thread = Thread(target=self.call_kinova, args=["Initialize"])
        initialize_thread.start()
        self.interface.start_render_loop()

    def call_sim(self) -> None:
        """Call a simulation service."""
        request = SimSrv.Request()
        request.name = "ToggleAutomove"
        self.sim_client.call(request)

    def call_kinova(self, name: str) -> None:
        """Call a kinova service."""
        self.interface.state.mode = "waiting"
        self.interface.update_control()
        request = KinSrv.Request()
        request.name = name
        self.kinova_client.call_async(request)

    def command_dingo(self) -> None:
        """Send a command to Dingo."""
        command = DinCmd()
        command.wheel_command = self.interface.wheel_torques
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

    def kin_sts(self, msg: KinSts) -> None:
        """Update the Kinova state."""
        for joint in self.interface.joints:
            joint.active = msg.joint_active[joint.index]
            joint.mode = msg.joint_mode[joint.index][:3]
            joint.ratio = msg.joint_ratio[joint.index]
            joint.fric_d = msg.joint_fric_d[joint.index]
            joint.fric_s = msg.joint_fric_s[joint.index]
        self.interface.state.mode = msg.mode
        self.interface.state.servoing = msg.servoing
        self.interface.state.comp_fric = msg.compensate_friction
        self.interface.state.move_tar = msg.automove_target
        self.interface.update_control()
        self.interface.update_state()

    def start_spin_loop(self) -> None:
        """Start the ros spin loop."""
        rclpy.spin(self)


def main(args: any = None) -> None:
    """Main."""
    rclpy.init(args=args)
    UserInterfaceNode()
    rclpy.shutdown()
    os._exit(0)


if __name__ == "__main__":
    main()
