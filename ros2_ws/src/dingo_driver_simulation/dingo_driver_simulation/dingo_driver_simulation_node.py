import rclpy
import os
import signal
import numpy as np
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
        torques = self.calculate_torques(msg.direction)
        sim_command.joint_tor = [torque * GAIN for torque in torques]
        self.pub_cmd.publish(sim_command)

    def calculate_torques(self, direction: list) -> list:
        """Calculate the required wheel torques to move in the given direction."""
        m = np.linalg.norm(direction)
        angle = np.arctan2(*direction)

        orientations = ["l", "r", "r", "l"]
        torques = []
        for n in range(4):
            orientation = orientations[n]
            torque = self.calculate_torque(angle, orientation) * m
            torques.append(torque)
        return torques

    def calculate_torque(self, angle: float, orientation: str) -> float:
        """Calculate the required wheel torque to match the given moving angle."""
        torques = [-1, -1, -1, 0, 1, 1, 1, 0, -1]
        angles = np.linspace(-np.pi, np.pi, 9)
        if orientation == "r":
            torques.reverse()
        return np.interp(angle, angles, torques)

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
