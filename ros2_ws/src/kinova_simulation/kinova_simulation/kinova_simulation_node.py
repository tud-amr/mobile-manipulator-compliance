import rclpy
import os
from rclpy.node import Node
from kinova_driver_msg.msg import KinovaFeedback, JointFeedback, Command
from kinova_driver_msg.srv import Service
from kinova.kortex_client_simulation import KortexClientSimulation
from controllers.state import State
from controllers.controllers import CompensateGravity
from threading import Thread


class KinovaSimulationNode(Node):
    """A node that starts the simulation of the kinova arm."""

    def __init__(self) -> None:
        super().__init__("kinova_driver_node")
        self.publisher = self.create_publisher(
            KinovaFeedback, "/kinova_driver/feedback", 10
        )
        self.subscription = self.create_subscription(
            Command, "/kinova_driver/command", self.callback, 10
        )
        self.service = self.create_service(
            Service, "/kinova/service", self.service_call
        )

        self.kortex_client = KortexClientSimulation()
        self.state = State(True, self.kortex_client.actuator_count)
        self.controller = CompensateGravity(self.state)
        self.kortex_client.feedback_callback = self.publish_feedback

        spin_thread = Thread(target=self.start_spin_loop)
        spin_thread.start()
        self.kortex_client.mujoco_viewer.start_simulation()

    def service_call(
        self, request: Service.Request, response: Service.Response
    ) -> Service.Response:
        """Connect service calls with kortex client."""
        match request.name:
            case "Home":
                self.kortex_client.home()
                response.mode = "HLC"
            case "Zero":
                self.kortex_client.zero()
                response.mode = "HLC"
            case "Retract":
                self.kortex_client.retract()
                response.mode = "HLC"
            case "Start LLC":
                self.kortex_client._start_LLC()
                response.mode = "LLC"
            case "Stop LLC":
                self.kortex_client._stop_LLC()
                response.mode = "HLC"
            case "Stop LLC Task":
                self.kortex_client._disconnect_LLC()
                response.mode = "LLC"
            case "Gravity":
                self.kortex_client.connect_LLC(self.controller, "current")
                response.mode = "LLC_task"
            case _:
                print("Service call is unknown.")

        return response

    def publish_feedback(self) -> None:
        """Publish the joint feedback."""
        feedback = KinovaFeedback()
        for n in range(self.kortex_client.actuator_count):
            joint_feedback = JointFeedback()
            joint_feedback.position = self.state.q[n] = self.kortex_client.get_position(n, False)
            joint_feedback.speed = self.state.dq[n] = self.kortex_client.get_velocity(n, False)
            joint_feedback.current = self.kortex_client.get_torque(n, False)
            setattr(feedback, f"joint{n}", joint_feedback)
        self.state.update()
        self.publisher.publish(feedback)

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
