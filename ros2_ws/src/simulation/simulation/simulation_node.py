import rclpy
from rclpy.node import Node
import os
from compliant_control.simulation.simulation import Simulation
from simulation_msg.msg import SimFdbk, SimCmd, SimCmdInc
from simulation_msg.srv import SimSrv
from threading import Thread


class SimulationNode(Node):
    """A node that starts the driver or a simulation of the Kinova arm."""

    def __init__(self) -> None:
        super().__init__("simulation_node")
        self.create_subscription(
            SimCmdInc,
            "/simulation/command_increment",
            self.command_increment,
            10,
        )
        self.create_subscription(SimCmd, "/simulation/command", self.command, 10)
        self.create_service(SimSrv, "/simulation/service", self.service_call)
        self.pub = self.create_publisher(SimFdbk, "/simulation/feedback", 10)
        self.spin_thread = Thread(target=self.start_spin_loop)
        self.sim = Simulation(self.callback)
        self.spin_thread.start()
        self.sim.start()

    def service_call(self, request: SimSrv.Request, response: SimSrv.Response) -> None:
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

    def command_increment(self, msg: SimCmdInc) -> None:
        """Control increment."""
        self.sim.ctrl_increment("Kinova", msg.type, msg.joint, msg.increment)

    def command(self, msg: SimCmd) -> None:
        """Update the simulation controllers."""
        match msg.robot:
            case "Kinova":
                self.sim.set_ctrl_value(msg.robot, "position", msg.joint_pos.data)
                self.sim.set_ctrl_value(msg.robot, "velocity", msg.joint_vel.data)
                self.sim.set_ctrl_value(msg.robot, "torque", msg.joint_tor.data)
            case "Dingo":
                self.sim.set_ctrl_value(msg.robot, "torque", msg.joint_tor.data)

    def callback(self) -> None:
        """Callback that is called every simulation step."""
        feedback = SimFdbk()
        feedback.joint_pos.data = self.sim.get_sensor_feedback("Kinova", "position")
        feedback.joint_vel.data = self.sim.get_sensor_feedback("Kinova", "velocity")
        feedback.joint_tor.data = self.sim.get_sensor_feedback("Kinova", "torque")
        feedback.wheel_pos.data = self.sim.get_sensor_feedback("Dingo", "position")
        feedback.wheel_vel.data = self.sim.get_sensor_feedback("Dingo", "velocity")
        feedback.wheel_tor.data = self.sim.get_sensor_feedback("Dingo", "torque")
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
