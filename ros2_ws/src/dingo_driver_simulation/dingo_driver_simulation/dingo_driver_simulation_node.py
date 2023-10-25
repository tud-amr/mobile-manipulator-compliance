import rclpy
import os
from rclpy.node import Node

from dingo_driver_msg.msg import DinFdbk, DinCmd
from simulation_msg.msg import SimFdbk, SimCmd

WHEELS = 4


class DingoDriverSimulationNode(Node):
    """A node that starts the driver or a simulation of the Kinova arm."""

    def __init__(self) -> None:
        super().__init__("dingo_driver_simulation_node")
        self.create_subscription(SimFdbk, "/sim/fdbk", self.sim_fdbk, 10)
        self.pub_fdbk = self.create_publisher(DinFdbk, "/dingo/fdbk", 10)
        self.create_subscription(DinCmd, "/dingo/cmd", self.cmd, 10)
        self.pub_cmd = self.create_publisher(SimCmd, "/sim/cmd", 10)

        self.start_spin_loop()

    def sim_fdbk(self, msg: SimFdbk) -> None:
        """Process the feedback from the simulation."""
        feedback = DinFdbk()
        feedback.wheel_pos = msg.wheel_pos
        feedback.wheel_vel = msg.wheel_vel
        feedback.wheel_tor = msg.wheel_tor
        self.pub_fdbk.publish(feedback)

    def cmd(self, msg: DinCmd) -> None:
        """Send dingo command to simulation."""
        command_msg = SimCmd()
        command_msg.robot = "Dingo"
        command_msg.joint_tor = msg.wheel_command
        self.pub_cmd.publish(command_msg)

    def start_spin_loop(self) -> None:
        """Start the ros spin loop."""
        rclpy.spin(self)


def main(args: any = None) -> None:
    """Main."""
    rclpy.init(args=args)
    DingoDriverSimulationNode()
    rclpy.shutdown()
    os._exit(0)


if __name__ == "__main__":
    main()
