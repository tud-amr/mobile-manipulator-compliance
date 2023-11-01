import rclpy
import os
import signal
from rclpy.node import Node
from threading import Thread

from kinova_driver_msg.msg import KinFdbk, KinCmd
from kinova_driver_msg.srv import KinSrv

from compliant_control.kinova.kortex_client import KortexClient
from compliant_control.kinova.kortex_client_simulation import KortexClientSimulation
from compliant_control.kinova.utilities import DeviceConnection, ip_available

JOINTS = 6


class KinovaServiceNode(Node):
    """Handles the services requests."""

    def __init__(self, kortex_client: KortexClient) -> None:
        super().__init__("kinova_service_node")
        self.kortex_client = kortex_client
        self.create_service(KinSrv, "/kinova/srv", self.service_call)

    def service_call(
        self, request: KinSrv.Request, response: KinSrv.Response
    ) -> KinSrv.Response:
        """Execute service call."""
        print(f"KIN CALL: {request.name}")
        match request.name:
            case "Refresh":
                pass
            case "Home":
                self.kortex_client.home()
            case "Zero":
                self.kortex_client.zero()
            case "Retract":
                self.kortex_client.retract()
            case "Pref":
                self.kortex_client.pref()
            case "Start LLC":
                self.kortex_client.start_LLC()
            case "Stop LLC":
                self.kortex_client.stop_LLC()
            case "Start LLC Task":
                self.kortex_client.connect_LLC()
            case "Stop LLC Task":
                self.kortex_client.disconnect_LLC()
            case "Clear Faults":
                self.kortex_client.clear_faults()
            case _ if request.name in [str(n) for n in range(JOINTS)]:
                self.kortex_client.toggle_active(int(request.name))
            case _:
                print(f"Service call {request.name} is unknown.")

        response.mode = self.kortex_client.mode
        response.servoing_mode = self.kortex_client.get_servoing_mode()
        response.active = self.kortex_client.joint_active
        for n in range(self.kortex_client.actuator_count):
            response.control_mode.append(self.kortex_client.get_control_mode(n))
        return response


class KinovaDriverNode(Node):
    """Handles joint feedback and commands."""

    def __init__(self, kortex_client: KortexClient) -> None:
        super().__init__("kinova_driver_node")
        self.kortex_client = kortex_client
        self.create_subscription(KinCmd, "kinova/cmd", self.cmd, 10)
        self.pub = self.create_publisher(KinFdbk, "/kinova/fdbk", 10)
        kortex_client.feedback_callback = self.pub_fdbk

    def cmd(self, msg: KinCmd) -> None:
        """Process the command."""
        self.kortex_client.copy_feedback_to_command()
        self.kortex_client.set_command(msg.joint_command)

    def pub_fdbk(self) -> None:
        """Publish the joint feedback of Kinova arm."""
        feedback = KinFdbk()
        feedback.update_rate = self.kortex_client.get_update_rate()
        for n in range(self.kortex_client.actuator_count):
            position = self.kortex_client.get_position(n, False)
            velocity = self.kortex_client.get_velocity(n, False)
            torque = self.kortex_client.get_current(n, False)
            feedback.joint_pos.append(position)
            feedback.joint_vel.append(velocity)
            feedback.joint_tor.append(torque)
        self.pub.publish(feedback)


def start(kortex_client: KortexClient, sim: bool = False) -> None:
    """Initialize the nodes and start the kortex loop."""
    executor: rclpy.Executor = rclpy.executors.MultiThreadedExecutor()

    feedback = KinovaDriverNode(kortex_client)
    service = KinovaServiceNode(kortex_client)

    if sim:
        executor.add_node(kortex_client)
    executor.add_node(feedback)
    executor.add_node(service)

    spin_thread = Thread(target=executor.spin)
    spin_thread.start()

    signal.signal(signal.SIGINT, kortex_client.stop)
    kortex_client.start()


def main(args: any = None) -> None:
    """Main."""
    rclpy.init(args=args)
    if not ip_available():
        start(KortexClientSimulation(), sim=True)
    else:
        with DeviceConnection.createTcpConnection() as router, DeviceConnection.createUdpConnection() as real_time_router:
            start(KortexClient(router=router, real_time_router=real_time_router))
    rclpy.shutdown()
    os._exit(0)


if __name__ == "__main__":
    main()
