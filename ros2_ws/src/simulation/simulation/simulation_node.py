import rclpy
from rclpy.node import Node
import os
from compliant_control.mujoco.viewer import Viewer
from simulation_msg.msg import SimFdbk, SimCmd, SimCmdInc
from simulation_msg.srv import SimSrv
from threading import Thread


class SimulationNode(Node):
    """A node that starts a simulation of the robot."""

    def __init__(self) -> None:
        super().__init__("simulation_node")
        self.create_subscription(SimCmdInc, "/sim/cmd_inc", self.cmd_inc, 10)
        self.create_subscription(SimCmd, "/sim/cmd", self.cmd, 10)
        self.create_service(SimSrv, "/sim/srv", self.srv)
        self.pub = self.create_publisher(SimFdbk, "/sim/fdbk", 10)
        self.spin_thread = Thread(target=self.start_spin_loop)
        self.sim = Viewer("simulation", self.pub_fdbk)
        self.spin_thread.start()
        self.sim.start()

    def srv(self, request: SimSrv.Request, response: SimSrv.Response) -> None:
        """Execute service call."""
        match request.name:
            case "KinovaPositionMode":
                self.sim.change_mode("position", request.arg)
            case "KinovaTorqueMode":
                self.sim.change_mode("torque", request.arg)
            case "ToggleAutomove":
                self.sim.toggle_automove_target()
        response.success = True
        return response

    def cmd_inc(self, msg: SimCmdInc) -> None:
        """Send command increment to simulation."""
        self.sim.ctrl_increment("Kinova", msg.type, msg.joint, msg.increment)

    def cmd(self, msg: SimCmd) -> None:
        """Send command to simulation."""
        match msg.robot:
            case "Kinova" if self.sim.kinova:
                self.sim.set_ctrl_value(msg.robot, "position", msg.joint_pos)
                self.sim.set_ctrl_value(msg.robot, "velocity", msg.joint_vel)
                self.sim.set_ctrl_value(msg.robot, "torque", msg.joint_tor)
            case "Dingo" if self.sim.dingo:
                self.sim.set_ctrl_value(msg.robot, "torque", msg.joint_tor)

    def pub_fdbk(self) -> None:
        """Publish the simulation feedback."""
        feedback = SimFdbk()
        if self.sim.kinova:
            feedback.joint_pos = self.sim.get_sensor_feedback("Kinova", "position")
            feedback.joint_vel = self.sim.get_sensor_feedback("Kinova", "velocity")
            feedback.joint_tor = self.sim.get_sensor_feedback("Kinova", "torque")
        if self.sim.dingo:
            feedback.wheel_pos = self.sim.get_sensor_feedback("Dingo", "position")
            feedback.wheel_vel = self.sim.get_sensor_feedback("Dingo", "velocity")
            feedback.wheel_tor = self.sim.get_sensor_feedback("Dingo", "torque")
        self.pub.publish(feedback)

    def start_spin_loop(self) -> None:
        """Start the ros spin loop."""
        rclpy.spin(self)


def main(args: any = None) -> None:
    """Main."""
    rclpy.init(args=args)
    SimulationNode()
    rclpy.shutdown()
    os._exit(0)


if __name__ == "__main__":
    main()
