import rclpy
import os
import signal
import numpy as np
import subprocess
from rclpy.node import Node
from kinova_driver_msg.msg import KinovaFeedback, JointFeedback, KinovaState, JointState
from kinova_driver_msg.srv import Service
from dingo_driver_msg.msg import DingoFeedback, DingoCommand
from compliant_control.kinova.kortex_client_simulation import KortexClientSimulation
from compliant_control.kinova.utilities import DeviceConnection, DEFAULT_IP
from compliant_control.kinova.kortex_client import KortexClient
from compliant_control.kinova.mujoco_viewer import MujocoViewer
from mujoco_viewer_msg.msg import MujocoFeedback, MujocoCommand
from compliant_control.controllers.state import State
from compliant_control.controllers.controllers import Controllers, Controller
from compliant_control.controllers.calibration import Calibrations
from threading import Thread
from std_msgs.msg import MultiArrayDimension
import mujoco


class KinovaDriverNode(Node):
    """A node that starts the driver or a simulation of the Kinova arm."""

    def __init__(self) -> None:
        super().__init__("kinova_driver_node")
        self.create_subscription(MujocoFeedback, "/mujoco/feedback", self.callback, 10)
        self.mujoco_pub = self.create_publisher(MujocoCommand, "/mujoco/command", 10)
        self.feedback_pub = self.create_publisher(
            KinovaFeedback, "/kinova/feedback", 10
        )
        self.state_pub = self.create_publisher(KinovaState, "/kinova/state", 10)
        self.create_service(Service, "/kinova/service", self.service_call)

        self.dingo_pub = self.create_publisher(DingoFeedback, "/dingo/feedback", 10)
        self.create_subscription(
            DingoCommand, "/dingo/command", self.dingo_callback, 10
        )

        self.mujoco_viewer = MujocoViewer()
        self.spin_thread = Thread(target=self.start_spin_loop)

        if self.ip_available():
            self.start_driver()
        else:
            print("Kinova arm not found, starting simulation...")
            self.start_simulation()

    def dingo_callback(self, msg: DingoCommand) -> None:
        """Dingo command callback."""
        for fr in ["f", "r"]:
            for lr in ["l", "r"]:
                idx = mujoco.mj_name2id(
                    self.mujoco_viewer.model,
                    mujoco.mjtObj.mjOBJ_ACTUATOR,
                    f"D_MA_{(fr+lr).upper()}",
                )
                self.mujoco_viewer.data.ctrl[idx] = getattr(msg, fr + lr)

    def start_driver(self) -> None:
        """Start the driver for the Kinova arm."""
        with DeviceConnection.createTcpConnection() as router, DeviceConnection.createUdpConnection() as real_time_router:
            self.kortex_client = KortexClient(
                router=router, real_time_router=real_time_router
            )

            self.kortex_client.feedback_callback = self.publish_feedback
            self.state = State(False, self.kortex_client.actuator_count)
            self.controllers = Controllers(self.state)
            self.calibrations = Calibrations(self.state, self.kortex_client)
            self.publish_state()

            signal.signal(signal.SIGINT, self.kortex_client.stop_refresh_loop)
            self.spin_thread.start()
            self.kortex_client.start_refresh_loop()

    def start_simulation(self) -> None:
        """Start a simulation of the Kinova arm."""
        self.kortex_client = KortexClientSimulation(self.mujoco_viewer)

        self.kortex_client.feedback_callback = self.publish_feedback
        self.state = State(True, self.kortex_client.actuator_count)
        self.controllers = Controllers(self.state)
        self.calibrations = Calibrations(self.state, self.kortex_client)
        self.publish_state()

        kortex_thread = Thread(target=self.kortex_client.start_refresh_loop)
        kortex_thread.start()
        signal.signal(signal.SIGINT, self.mujoco_viewer.stop_simulation)
        self.spin_thread.start()
        self.mujoco_viewer.start_simulation()

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
                self.kortex_client._connect_LLC(
                    self.controllers.compensate_gravity_and_friction
                )
            case "Impedance":
                msg = MujocoCommand()
                msg.target.data = list(self.state.x)
                self.mujoco_pub.publish(msg)
                self.kortex_client._connect_LLC(self.controllers.impedance)
            case "Cartesian Impedance":
                msg = MujocoCommand()
                msg.target.data = list(self.state.x)
                self.mujoco_pub.publish(msg)
                self.kortex_client._connect_LLC(self.controllers.cartesian_impedance)
            case "HL Calibration":
                self.calibrations.high_level.calibrate_all_joints()
            case "LL Calibration":
                self.calibrations.low_level.calibrate_all_joints()
            case "Compensate friction":
                Controller.toggle_CF()
            case "Clear Faults":
                self.kortex_client.clear_faults()
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

        feedback = DingoFeedback()
        idx = mujoco.mj_name2id(
            self.mujoco_viewer.model, mujoco.mjtObj.mjOBJ_JOINT, "D_J_B"
        )
        idpos = self.mujoco_viewer.model.jnt_qposadr[idx]
        feedback.base_pos_x = self.mujoco_viewer.data.qpos[idpos]
        feedback.base_pos_y = self.mujoco_viewer.data.qpos[idpos + 1]
        feedback.base_rot_z = self.mujoco_viewer.data.qpos[idpos + 6]
        self.dingo_pub.publish(feedback)

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
        state.compensate_friction = Controller.get_CF()
        state.automove_target = self.mujoco_viewer.automove_target
        self.state_pub.publish(state)

    def callback(self, msg: MujocoFeedback) -> None:
        """Mujoco callback."""
        x: MultiArrayDimension = msg.perturbations.layout.dim[0]
        y: MultiArrayDimension = msg.perturbations.layout.dim[1]
        self.mujoco_viewer.data.xfrc_applied = np.array(msg.perturbations.data).reshape(
            (x.size, y.size)
        )
        self.mujoco_viewer.update_target(np.array(msg.target.data))

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
