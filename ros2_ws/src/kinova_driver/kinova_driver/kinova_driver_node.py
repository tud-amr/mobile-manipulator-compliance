import rclpy
import os
import signal
import time
import subprocess
import numpy as np
from rclpy.node import Node
from threading import Thread

from kinova_driver_msg.msg import KinFdbk, KinSts, KinTar
from kinova_driver_msg.srv import KinSrv

from simulation_msg.srv import SimSrv

from compliant_control.kinova.kortex_client import KortexClient
from compliant_control.kinova.kortex_client_simulation import KortexClientSimulation
from compliant_control.kinova.utilities import DeviceConnection, DEFAULT_IP

from compliant_control.controllers.state import State
from compliant_control.controllers.controllers import Controllers, Controller
from compliant_control.controllers.calibration import Calibrations


class KinovaDriverNode(Node):
    """A node that starts the driver or a simulation of the Kinova arm."""

    def __init__(self) -> None:
        super().__init__("kinova_driver_node")
        self.create_subscription(KinTar, "/kinova/tar", self.update_target, 10)
        self.create_service(KinSrv, "/kinova/srv", self.service_call)
        self.state_pub = self.create_publisher(KinSts, "/kinova/sts", 10)
        self.pub = self.create_publisher(KinFdbk, "/kinova/fdbk", 10)
        self.sim_srv = self.create_client(SimSrv, "/sim/srv")

        self.state = State()
        self.controllers = Controllers(self.state)

        spin_thread = Thread(target=self.start_spin_loop)
        spin_thread.start()

        if self.ip_available():
            self.state.simulation = False
            self.start_driver()
        else:
            self.state.simulation = True
            self.start_simulation()

    def start_simulation(self) -> None:
        """Start a simulation of the Kinova arm."""
        print("Kinova arm not found, starting simulation...")
        self.kortex_client = KortexClientSimulation(self)
        self.start_kortex()

    def start_driver(self) -> None:
        """Start the driver for the Kinova arm."""
        with DeviceConnection.createTcpConnection() as router, DeviceConnection.createUdpConnection() as real_time_router:
            self.kortex_client = KortexClient(
                router=router, real_time_router=real_time_router
            )
            self.start_kortex()

    def start_kortex(self) -> None:
        """Start kortex."""
        self.kortex_client.feedback_callback = self.pub_fdbk
        self.calibrations = Calibrations(self.state, self.kortex_client)
        signal.signal(signal.SIGINT, self.kortex_client.stop_refresh_loop)
        self.publish_state()
        self.kortex_client.start_refresh_loop()

    def service_call(
        self, request: KinSrv.Request, response: KinSrv.Response
    ) -> KinSrv.Response:
        """Execute service call in new thread."""
        thread = Thread(target=self.execute_service, args=[request.name])
        thread.start()
        return response

    def execute_service(self, name: str) -> None:
        """Execute service call."""
        match name:
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
                self.kortex_client._connect_LLC(
                    self.controllers.compensate_gravity_and_friction
                )
            case "Impedance":
                self.reset_target()
                self.kortex_client._connect_LLC(self.controllers.impedance)
            case "Cartesian Impedance":
                self.reset_target()
                self.kortex_client._connect_LLC(self.controllers.cartesian_impedance)
            case "HL Calibration":
                self.calibrations.high_level.calibrate_all_joints()
            case "LL Calibration":
                self.calibrations.low_level.calibrate_all_joints()
            case "Compensate friction":
                Controller.toggle_CF()
            case "Clear Faults":
                self.kortex_client.clear_faults()
            case _ if "Tog" in name:
                self.state.toggle_joint(int(name[-1]))
            case _:
                print(f"Service call {name} is unknown.")

        self.publish_state()

    def reset_target(self) -> None:
        """Reset the target."""
        request = SimSrv.Request()
        request.name = "ResetTarget"
        future = self.sim_srv.call_async(request)
        while not future.done():
            time.sleep(0.1)

    def update_target(self, msg: KinTar) -> None:
        """Update the target."""
        self.state.target = np.array(msg.target)

    def pub_fdbk(self) -> None:
        """Publish the joint feedback of Kinova arm."""
        feedback = KinFdbk()
        feedback.update_rate = self.kortex_client.get_update_rate()
        for n in range(self.kortex_client.actuator_count):
            self.state.q[n] = self.kortex_client.get_position(n, False)
            self.state.dq[n] = self.kortex_client.get_velocity(n, False)
            torque = self.kortex_client.get_current(n, False)
            feedback.joint_pos.append(self.state.q[n])
            feedback.joint_vel.append(self.state.dq[n])
            feedback.joint_tor.append(torque)
        self.state.update()
        self.pub.publish(feedback)

    def publish_state(self) -> None:
        """Publish the status of the Kinova arm."""
        status = KinSts()
        for n in range(self.kortex_client.actuator_count):
            status.joint_active.append(self.state.active[n])
            status.joint_mode.append(self.kortex_client.get_control_mode(n))
            status.joint_ratio.append(self.state.get_ratio(n))
            status.joint_fric_s.append(self.state.static_frictions[n])
            status.joint_fric_d.append(self.state.dynamic_frictions[n])
        status.mode = self.kortex_client.mode
        status.servoing = self.kortex_client.get_servoing_mode()
        status.compensate_friction = Controller.get_CF()
        self.state_pub.publish(status)

    def start_spin_loop(self) -> None:
        """Start the ros spin loop."""
        rclpy.spin(self)

    def ip_available(self) -> bool:
        """Check if robot is available."""
        return (
            subprocess.call(
                "ping -c 1 -W 0.1 " + DEFAULT_IP,
                shell=True,
                stdout=subprocess.DEVNULL,
            )
            == 0
        )


def main(args: any = None) -> None:
    """Main."""
    rclpy.init(args=args)
    KinovaDriverNode()
    rclpy.shutdown()
    os._exit(0)


if __name__ == "__main__":
    main()
