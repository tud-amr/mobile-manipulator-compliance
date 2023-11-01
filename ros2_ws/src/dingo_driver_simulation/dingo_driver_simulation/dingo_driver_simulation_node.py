import rclpy
import os
import signal
from rclpy.node import Node

from dingo_driver_msg.msg import DinFdbk, DinCmd
from simulation_msg.msg import SimFdbk, SimCmd

WHEELS = 4
GAIN = 3


class DingoDriverSimulationNode(Node):
    """A node that starts the driver or a simulation of the Kinova arm."""

    def __init__(self) -> None:
        super().__init__("dingo_driver_simulation_node")
        self.create_subscription(SimFdbk, "/sim/fdbk", self.sim_fdbk, 10)
        self.pub_fdbk = self.create_publisher(DinFdbk, "/dingo/fdbk", 10)
        self.create_subscription(DinCmd, "/dingo/cmd", self.cmd, 10)
        self.pub_cmd = self.create_publisher(SimCmd, "/sim/cmd", 10)

        signal.signal(signal.SIGINT, self.stop)
        self.start()

    def sim_fdbk(self, msg: SimFdbk) -> None:
        """Process the feedback from the simulation."""
        feedback = DinFdbk()
        feedback.wheel_pos = msg.wheel_pos
        feedback.wheel_vel = msg.wheel_vel
        feedback.wheel_tor = msg.wheel_tor
        self.pub_fdbk.publish(feedback)

    def cmd(self, msg: DinCmd) -> None:
        """Send dingo command to simulation."""
        sim_command = SimCmd()
        sim_command.robot = "Dingo"
        torques = msg.wheel_command
        sim_command.joint_tor = [torque * GAIN for torque in torques]
        self.pub_cmd.publish(sim_command)

    def start(self) -> None:
        """Start the ros spin loop."""
        self.active = True
        while self.active:
            rclpy.spin_once(self, timeout_sec=0)

    def stop(self, *args: any) -> None:
        """Stop the ros spin loop."""
        self.active = False


def main(args: any = None) -> None:
    """Main."""
    rclpy.init(args=args)
    DingoDriverSimulationNode()
    rclpy.shutdown()
    os._exit(0)


if __name__ == "__main__":
    main()
