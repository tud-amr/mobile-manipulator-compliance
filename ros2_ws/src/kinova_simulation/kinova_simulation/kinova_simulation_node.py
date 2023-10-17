import rclpy
import os
from rclpy.node import Node
from kinova_driver_msg.msg import (
    KinovaFeedback,
    JointFeedback,
    KinovaState,
    JointState,
    Command,
)
from kinova_driver_msg.srv import Service
from kinova.kortex_client_simulation import KortexClientSimulation
from kinova.mujoco_viewer import MujocoViewer
from controllers.state import State
from controllers.controllers import Controllers
from controllers.calibration import Calibrations
from threading import Thread


class KinovaSimulationNode(Node):
    """A node that starts the simulation of the kinova arm."""

    def __init__(self) -> None:
        super().__init__("kinova_driver_node")
        self.feedback_pub = self.create_publisher(
            KinovaFeedback, "/kinova/feedback", 10
        )
        self.state_pub = self.create_publisher(KinovaState, "/kinova/state", 10)
        self.create_service(Service, "/kinova/service", self.service_call)

        self.mujoco_viewer = MujocoViewer()
        self.kortex_client = KortexClientSimulation(self.mujoco_viewer)
        self.state = State(True, self.kortex_client.actuator_count)
        self.controllers = Controllers(self.state)
        self.calibrations = Calibrations(self.state, self.kortex_client)
        self.kortex_client.feedback_callback = self.publish_feedback

        spin_thread = Thread(target=self.start_spin_loop)
        spin_thread.start()
        self.publish_state()
        self.kortex_client.mujoco_viewer.start_simulation()

    def service_call(
        self, request: Service.Request, response: Service.Response
    ) -> Service.Response:
        """Connect service calls with kortex client."""
        match request.name:
            case "Initialize":
                self.publish_state()
            case "Home":
                self.kortex_client.home()
            case "Zero":
                self.kortex_client.zero()
            case "Retract":
                self.kortex_client.retract()
            case "Start LLC":
                self.kortex_client._start_LLC()
            case "Stop LLC":
                self.kortex_client._stop_LLC()
            case "Stop LLC Task":
                self.kortex_client._disconnect_LLC()
            case "Gravity":
                self.kortex_client._connect_LLC(self.controllers.compensate_gravity)
            case "Impedance":
                self.mujoco_viewer.update_marker("target", self.state.x)
                self.kortex_client._connect_LLC(self.controllers.impedance)
            case "Cartesian Impedance":
                self.mujoco_viewer.update_marker("target", self.state.x)
                self.kortex_client._connect_LLC(self.controllers.cartesion_impedance)
            case "HL Calibration":
                self.calibrations.high_level.calibrate_all_joints()
            case "LL Calibration":
                self.calibrations.low_level.calibrate_all_joints()
            case _ if "Toggle" in request.name:
                self.state.toggle_joint(int(request.name[-1]))
            case _:
                print(f"Service call {request.name} is unknown.")

        self.publish_state()
        response.mode = self.kortex_client.mode
        return response

    def publish_feedback(self) -> None:
        """Publish the joint feedback."""
        feedback = KinovaFeedback()
        feedback.update_rate = self.kortex_client.get_update_rate()
        for n in range(self.kortex_client.actuator_count):
            joint_feedback = JointFeedback()
            joint_feedback.position = self.state.q[n] = self.kortex_client.get_position(
                n, False
            )
            joint_feedback.speed = self.state.dq[n] = self.kortex_client.get_velocity(
                n, False
            )
            joint_feedback.current = self.kortex_client.get_torque(n, False)
            setattr(feedback, f"joint{n}", joint_feedback)
        self.feedback_pub.publish(feedback)
        self.state.target = self.mujoco_viewer.target
        self.state.update()
        self.mujoco_viewer.update_marker("end_effector", self.state.x)

    def publish_state(self) -> None:
        """Publish the joint state."""
        state = KinovaState()
        for n in range(self.kortex_client.actuator_count):
            joint_state = JointState()
            joint_state.active = self.state.active[n]
            joint_state.mode = self.kortex_client.get_control_mode(n)
            joint_state.ratio = float(self.state.get_ratio(n))
            joint_state.fric_d = self.state.dynamic_frictions[n]
            joint_state.fric_s = self.state.static_frictions[n]
            setattr(state, f"joint{n}", joint_state)
        state.mode = self.kortex_client.mode
        state.servoing = self.kortex_client.get_servoing_mode()
        self.state_pub.publish(state)

    def callback(self, msg: Command) -> None:
        """Command callback."""

    def start_spin_loop(self) -> None:
        """Start the ros spin loop."""
        rclpy.spin(self)


def main(args: any = None) -> None:
    """Main."""
    rclpy.init(args=args)
    KinovaSimulationNode()
    rclpy.shutdown()
    os._exit(0)


if __name__ == "__main__":
    main()
