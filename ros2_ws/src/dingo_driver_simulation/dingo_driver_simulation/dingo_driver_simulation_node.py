import rclpy
import os
from rclpy.node import Node

from dingo_driver_msg.msg import DingoFeedback, WheelFeedback, DingoCommand
from simulation_msg.msg import SimFdbk, SimCmd

WHEELS = 4


class DingoDriverSimulationNode(Node):
    """A node that starts the driver or a simulation of the Kinova arm."""

    def __init__(self) -> None:
        super().__init__("dingo_driver_simulation_node")
        self.create_subscription(SimFdbk, "/sim/fdbk", self.sim_fdbk, 10)
        self.pub_fdbk = self.create_publisher(DingoFeedback, "/dingo/feedback", 10)
        self.create_subscription(DingoCommand, "/dingo/command", self.dingo_command, 10)
        self.pub_cmd = self.create_publisher(SimCmd, "/sim/cmd", 10)

        self.start_spin_loop()

    def sim_fdbk(self, msg: SimFdbk) -> None:
        """Process the feedback from the simulation."""
        dingo_feedback = DingoFeedback()
        n = 0
        for FR in ["front", "rear"]:
            for LR in ["left", "right"]:
                wheel_feedback = WheelFeedback()
                wheel_feedback.position = msg.wheel_pos.data[n]
                wheel_feedback.speed = msg.wheel_vel.data[n]
                wheel_feedback.current = msg.wheel_tor.data[n]
                setattr(dingo_feedback, f"{FR}_{LR}_wheel", wheel_feedback)
                n += 1
        self.pub_fdbk.publish(dingo_feedback)

    def dingo_command(self, msg: DingoCommand) -> None:
        """Send dingo command to simulation."""
        command_msg = SimCmd()
        command_msg.robot = "Dingo"
        for FR in ["f", "r"]:
            for LR in ["l", "r"]:
                command_msg.joint_tor.data.append(getattr(msg, FR + LR))
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
