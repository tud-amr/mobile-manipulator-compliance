from __future__ import annotations
import rospy
import os
import time
import signal
import sys
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Bool
from scipy.spatial.transform import Rotation as R
from threading import Thread
import numpy as np
from compliant_control.control.state import rotMatrix_to_quaternion
from compliant_control.control.state import State
from compliant_control.dingo.dingo_driver import DingoDriver
from compliant_control.kinova.kortex_client import KortexClient
from compliant_control.kinova.utilities import DeviceConnection

# from compliant_control.control.calibration import Calibration
from sensor_msgs.msg import Joy
from user_interface_msg.msg import Ufdbk, Ucmd, Ustate, Utarget, Record, Data

PUBLISH_RATE = 100
D_MAX_RESET = 0.01


class ControlInterfaceNode:
    """A node that starts the control interface of the robot."""

    def __init__(self, args) -> None:
        rospy.init_node("control_interface_node")
        self.simulate = "--simulate" in args
        self.base_on = "--base_on" in args #when argument is given, the base control is done via normal velocity control
        print("self.base_on:", self.base_on)
        self.base_compliant = "--base" in args #when this argument is given, also the base is made compliant
        self.platform_lidar_height = 0.314 #now hardcoded, should be imported from dinova.xacro in dinova_description
        self.emergency_switch_pressed = False
        self.fk_position = None
        
        self.pub_fdbk = rospy.Publisher("compliant/feedback", Ufdbk, queue_size=10)
        self.pub_state = rospy.Publisher("compliant/state", Ustate, queue_size=10)
        self.pub_record = rospy.Publisher("compliant/record", Record, queue_size=10)
        # self.pub_calibration = rospy.Publisher("compliant/calibration", Data, queue_size=10)
        self.pub_current_pose = rospy.Publisher("compliant/current_pose", PoseStamped, queue_size=1)
        self.pub_joint_states = rospy.Publisher("kinova/joint_states", JointState, queue_size=1)
        rospy.Subscriber("bluetooth_teleop/joy", Joy, self.callback_emergency_switch)
        rospy.Subscriber("compliant/command", Ucmd, self.handle_input, queue_size=10)
        rospy.Subscriber("compliant/make_compliant", Bool, self.make_compliant, queue_size=1)
        rospy.Subscriber("compliant/make_compliant_joint", Bool, self.make_compliant_joint, queue_size=1)
        rospy.Subscriber("compliant/target", Utarget, self.update_target, queue_size=10)
        rospy.Subscriber("compliant/set_stiffness", Float32MultiArray, self.update_stiffness, queue_size=10)
        rospy.Subscriber("compliant/set_stiffness_joints", Float32MultiArray, self.update_stiffness_joints, queue_size=10)
        rospy.Subscriber("compliant/desired_pose", Pose, self.desired_pose_target_callback, queue_size=10)
        rospy.Subscriber("compliant/desired_joints", JointState, self.desired_joints_target_callback, queue_size=10)
        rospy.Subscriber("compliant/fk/current_pose", PoseStamped, self.fk_callback, queue_size=10)
        self.base_vicon_pose= [0, 0, 0] #[x, y, theta]
        rospy.Subscriber("dinova/omni_states_vicon", JointState, self.vicon_base_callback, queue_size=10)
        self.lidar = rospy.get_param('lidar', False)
        
        self.automove_target = False
        self.state = State(self.simulate)
        self._joint_state = JointState()
        self._joint_state.name = [f'joint_{i}' for i in range(6)]

        if self.simulate:
            self.start_simulation()
        else:
            self.start_robot()

    def make_compliant(self, msg: Bool):
        if msg.data: 
          self.kinova.pref()
          rospy.loginfo("Waiting to reach pref position.")
          self.state.reset_target()
          time.sleep(5)
          self.kinova.start_LLC()
          self.kinova.connect_LLC()
          self.state.controller.toggle('arm')
          if self.base_compliant:
            self.state.controller.toggle('base')
        else:
            self.state.controller.toggle('arm')
            if self.base_compliant:
                self.state.controller.toggle('base')
            self.kinova.disconnect_LLC()
            self.kinova.stop_LLC()
            self.kinova.pref()

    def make_compliant_joint(self, msg: Bool):
        if msg.data: 
          self.kinova.pref()
          rospy.loginfo("Waiting to reach pref position.")
          self.state.reset_target()
          time.sleep(5)
          self.kinova.start_LLC()
          self.kinova.connect_LLC()
          self.state.controller.toggle('arm_joint')
          if self.base_compliant:
            self.state.controller.toggle('base')
        else:
            self.state.controller.toggle('arm_joint')
            if self.base_compliant:
                self.state.controller.toggle('base')
            self.kinova.disconnect_LLC()
            self.kinova.stop_LLC()
            self.kinova.pref()

    def start_threads(self) -> None:
        """Start the threads."""
        spin_thread = Thread(target=self.start_spin_loop)
        spin_thread.start()

        publish_thread = Thread(target=self.start_publish_loop)
        publish_thread.start()

    def start_robot(self) -> None:
        """Start the robot."""
        if self.base_on:
            self.dingo = DingoDriver(self.state)
            self.dingo.log = rospy.loginfo
        self.start_threads()
        with DeviceConnection.createTcpConnection() as router, DeviceConnection.createUdpConnection() as real_time_router:
            self.kinova = KortexClient(
                self.state, router=router, real_time_router=real_time_router
            )
            self.kinova.log = rospy.loginfo
            # self.calibration = Calibration(
            #     self.state, self.kinova, self.publish_calibration
            # )
            # self.calibration.log = rospy.loginfo
            signal.signal(signal.SIGINT, self.kinova.stop)
            self.kinova.start()

    def start_simulation(self) -> None:
        """Start the simulation."""
        self.simulation = Simulation(self.state)
        self.dingo = DingoDriverSimulation(self.state, self.simulation)
        self.kinova = KortexClientSimulation(self.state, self.simulation)
        self.kinova.log = rospy.loginfo
        self.kinova.start_in_new_thread()
        # self.calibration = Calibration(
        #     self.state, self.kinova, self.publish_calibration
        # )
        # self.calibration.log = rospy.loginfo
        self.start_threads()
        self.simulation.start()

    def start_publish_loop(self) -> None:
        """Start a loop that publishes the feedback."""
        while not hasattr(self, "kinova"):
            time.sleep(0.5)
        rospy.loginfo("READY!")
        while True:
            self.publish_feedback()
            self.publish_record()
            self.publish_pose()
            self.publish_joint_state()
            time.sleep(1 / PUBLISH_RATE)

    def publish_feedback(self) -> None:
        """Publish feedback."""
        feedback = Ufdbk()
        feedback.kinova_pos = list(self.state.kinova_feedback.q)
        feedback.kinova_vel = list(self.state.kinova_feedback.dq)
        feedback.kinova_tor = list(self.state.kinova_feedback.c)
        feedback.kinova_rate = self.kinova.rate_counter.rate
        feedback.dingo_pos = list(self.state.dingo_feedback.q)
        feedback.dingo_vel = list(self.state.dingo_feedback.dq)
        feedback.dingo_tor = list(self.state.dingo_feedback.c)
        if self.base_on:
            feedback.dingo_rate = self.dingo.rate_counter.rate
        feedback.controller_rate = self.state.controller.rate_counter.rate
        feedback.mode = self.kinova.mode
        self.pub_fdbk.publish(feedback)

    def publish_joint_state(self):
        self._joint_state.header.stamp = rospy.Time.now()
        self._joint_state.position = self.state.kinova_feedback.q
        self._joint_state.velocity = self.state.kinova_feedback.dq
        self._joint_state.effort = self.state.kinova_feedback.c
        self.pub_joint_states.publish(self._joint_state)

    def publish_pose(self):
        list_state = list(self.state.x)
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = list_state[0]
        pose.pose.position.y = list_state[1]
        pose.pose.position.z = list_state[2]
        """
        pose.pose.orientation.x = list_state[3]
        pose.pose.orientation.y = list_state[4]
        pose.pose.orientation.z = list_state[5]
        pose.pose.orientation.w = list_state[6]
        """
        self.pub_current_pose.publish(pose)
        

    def publish_record(self) -> None:
        """Publish data to record.""" #todo: fix rotation base in recording
        msg = Record()
        pos_x = list(self.state.x)
        quat_x = list(self.state.quat)
        if self.lidar:
            z_correction = self.platform_lidar_height
        else:
            z_correction = 0.
        pos_x2, quat_x2 = self.get_rotated_pose(pos_x, quat_x, list(self.base_vicon_pose), z_correction)
        msg.pos_x = pos_x2
        msg.quat_x = quat_x2
        if self.fk_position is not None:
            msg.pos_fk = self.fk_position
            msg.quat_fk = self.fk_orientation
        msg.pos_q = list(self.state.kinova_feedback.q)
        msg.vel_q = list(self.state.kinova_feedback.dq)
        msg.pose_b = list(self.base_vicon_pose)
        # msg.pose_b = list(self.state.pose_base)
        msg.relative_target = list(self.state.target)
        msg.absolute_target = list(self.state.absolute_target)
        msg.time = [time.perf_counter()]
        self.pub_record.publish(msg)

    # def publish_calibration(self, data: np.ndarray) -> None:
    #     """Publish the data."""
    #     msg = Data()
    #     msg.data = list(data)
    #     self.pub_calibration.publish(msg)

    def handle_input(self, msg: Ucmd) -> None:
        """Handle user input."""
        cmd = msg.command
        if cmd == "Refresh":
            pass
        elif cmd == "Home":
            self.kinova.home()
        elif cmd == "Zero":
            self.kinova.zero()
        elif cmd == "Retract":
            self.kinova.retract()
        elif cmd == "Pref":
            self.kinova.pref()
        elif cmd == "Start HLT":
            if self.target_is_at_end_effector():
                self.kinova.start_HLT()
        elif cmd == "Stop HLT":
            self.kinova.stop_HLT()
        elif cmd == "Start LLC":
            self.kinova.start_LLC()
        elif cmd == "Stop LLC":
            self.kinova.stop_LLC()
        elif cmd == "Start LLC Task":
            if self.target_is_at_end_effector():
                self.kinova.connect_LLC()
        elif cmd == "Stop LLC Task":
            self.kinova.disconnect_LLC()
        elif cmd == "Clear":
            self.kinova.clear_faults()
        elif cmd == "Open":
            self.kinova._move_gripper(close=False)
        elif cmd == "Close":
            self.kinova._move_gripper(close=True)
        elif cmd == "Automove":
            self.toggle_automove_target()
        elif cmd == "Reset":
            if self.simulate:
                self.simulation.reset_target()
        elif cmd in [str(n) for n in range(self.kinova.actuator_count)]:
            self.kinova.toggle_active(int(cmd))
        elif cmd in ["grav", "fric", "arm", "null", "base"]:
            if cmd == "arm" and not self.target_is_at_end_effector():
                pass
            else:
                self.state.controller.toggle(cmd)
        elif cmd == "Move Dingo":
            self.state.controller.reset_base_command()
            self.state.controller.command_base_direction(msg.args, 0.6)
        # elif cmd == "Calibrate":
        #     self.calibration.calibrate_all()
        else:
            print(f"Service call {cmd} is unknown.")
        self.publish_state()

    def publish_state(self) -> None:
        """Publish the state of the robot."""
        state = Ustate()
        state.servoing_mode = self.kinova.mode
        state.comp_grav = self.state.controller.comp_grav
        state.comp_fric = self.state.controller.comp_fric
        state.imp_arm = self.state.controller.imp_arm
        state.imp_null = self.state.controller.imp_null
        state.imp_base = self.state.controller.imp_base

        state.active = self.kinova.joint_active
        state.mode = self.kinova.get_control_modes()
        state.ratio = list(self.state.ratios)
        state.friction = list(self.state.frictions)
        state.automove_target = self.automove_target

        self.pub_state.publish(state)

    def target_is_at_end_effector(self) -> None:
        """Reset the target."""
        if self.simulate:
            self.simulation.reset_target()
            self.state.target = self.state.x
        return np.linalg.norm(self.state.target - self.state.x) < D_MAX_RESET

    def desired_pose_target_callback(self, pose: Pose) -> None:
        self.state.target = np.array([
          pose.position.x,
          pose.position.y,
          pose.position.z,
        ])
        
    def desired_joints_target_callback(self, joint: JointState) -> None:
        self.state.target_q = np.array(joint.position)
        
    def fk_callback(self, pose_fk: PoseStamped) -> None:
        self.fk_position = [pose_fk.pose.position.x, pose_fk.pose.position.y, pose_fk.pose.position.z]
        self.fk_orientation = [pose_fk.pose.orientation.x, pose_fk.pose.orientation.y, pose_fk.pose.orientation.z, pose_fk.pose.orientation.w]
        
    def vicon_base_callback(self, msg: JointState):
        # this pose is (x, y, theta)
        self.base_vicon_pose = msg.position[0:3]
        
    def callback_emergency_switch(self, msg: Joy):
        if msg.buttons[4]:
            self.emergency_switch_pressed = True
            self.state.controller.return_emergency_switched_pressed(True)
        else:
            self.emergency_switch_pressed = False
        

    def update_target(self, msg: Utarget) -> None:
        """Update the target."""
        self.state.target = np.array(msg.relative_target)
        self.state.absolute_target = np.array(msg.absolute_target)
        self.state.pos_base = np.array(msg.pos_b)
        self.state.quat_base = np.array(msg.quat_b)
        
    def update_stiffness(self, msg: Float32MultiArray) -> None:
        """
        IMPORTANT, only update stiffness when in non-compliant mode!!
        """
        # """Update the stiffness."""
        Kd = np.diag(msg.data[0:3])
        Dd = np.diag(msg.data[3:6])
        print("updating stiffness!")
        # self.kinova.disconnect_LLC()
        self.state.controller.reset_param_cartesian_impedance(
            Kd=Kd,
            Dd=Dd) 
        # self.kinova.connect_LLC()
        
    def update_stiffness_joints(self, msg: Float32MultiArray) -> None:
        """
        IMPORTANT, only update stiffness when in non-compliant mode!!
        """
        # """Update the stiffness."""
        Kq = np.diag(msg.data[0:6])
        Dq = np.diag(msg.data[6:12])
        print("updating stiffness!")
        # self.kinova.disconnect_LLC()
        self.state.controller.reset_param_joints_impedance(
            Kq=Kq,
            Dq=Dq) 
        # self.kinova.connect_LLC()

    def toggle_automove_target(self) -> None:
        """Toggle automove of target."""
        if self.simulate:
            self.simulation.target_mover.toggle()
            self.automove_target = self.simulation.target_mover.active
        else:
            self.automove_target = not self.automove_target

    def step_callback(self) -> None:
        """Called every simulation step."""
        self.state.target = self.simulation.relative_target

    def start_spin_loop(self) -> None:
        """Start node spinning."""
        rospy.spin()
        
    def get_rotated_pose(self, pos_x, quat_x, xytheta_base, z_correction=0):
        """
        Creates a 3x3 rotation matrix for a rotation around the z-axis.
        """
        theta = xytheta_base[2]
        T_base = np.array([
            [np.cos(theta), -np.sin(theta), 0,      xytheta_base[0]],
            [np.sin(theta), np.cos(theta),  0,      xytheta_base[1]],
            [0,             0,              1,      z_correction],
            [0,             0,              0,      1]
        ])
        rotation_matrix_base = T_base[:3, :3]
        quaternion_base = rotMatrix_to_quaternion(rotation_matrix_base)
        pos_x_long = np.append(np.array(pos_x), np.array([1]))
        pos_x = (T_base @ pos_x_long)[:3]
        # R_ee = R.from_quat(quat_x).as_matrix()
        # R_ee_in_world = rotation_matrix_base @ R_ee
        # QUAT_X = rotMatrix_to_quaternion(R_ee_in_world)
        quat_x = self.state.controller.quat_product(quaternion_base, quat_x)
        return list(pos_x), quat_x

def main(args: any = None):
    """Main."""
    args = rospy.myargv(argv=sys.argv)
    ControlInterfaceNode(args)
    os._exit(0)


if __name__ == "__main__":
    main()
