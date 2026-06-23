#!/usr/bin/env python3
import rospy
import time
import pickle
import sys
import os
import numpy as np
from user_interface_msg.msg import Record, Ufdbk, Record
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import Pose
import copy

"""
has to be run beforehand:
    roslaunch launcher robot_lfd_kinova.launch
"""


class TrackerCompliant():
    def __init__(self): #, robot_name):
        # -- parameters --#
        self.robot_name = "dingo2"
        rospy.init_node("make_compliant_node")
        self.Kd = np.eye(3) #* 40
        self.Kd[0, 0] = 40
        self.Kd[1, 1] = 40
        self.Kd[2, 2] = 10
        self.Dd = np.eye(3) * 3
        self.desired_pose_offset = [0., 0., 0.1]
        self.pose_current = None
        self.position_desired = None
        self.quaternion_desired = None
        self.start = True
        self.end = False
        self.TARGET_RESET = False
        self.target = [0.3, 0.2, 0.4]
        self.stiffness_enabled = False
        self.mode = "Unknown"
        
        # -- subscribers -- 
        rospy.Subscriber('bluetooth_teleop/joy', Joy, self._callback_joystick, queue_size=10)
        rospy.Subscriber('compliant/feedback', Ufdbk, self._callback_feedback_mode, queue_size=10)
        rospy.Subscriber('compliant/record', Record, self._callback_record, queue_size=10)
        # rospy.Subscriber('compliant/desired_pose', Pose, self._callback_desired_pose, queue_size=10)
        
        # -- publishers --
        self.pub_mode = rospy.Publisher("compliant/make_compliant", Bool, queue_size=1)
        self.pub_desired_pose = rospy.Publisher("compliant/desired_pose", Pose, queue_size=1)
        self.pub_stiffness = rospy.Publisher("compliant/set_stiffness", Float32MultiArray, queue_size=1)
        
        #         # -- subscribers -- 
        # rospy.Subscriber('bluetooth_teleop/joy' , Joy, self._callback_joystick, queue_size=10)
        # rospy.Subscriber('compliant/feedback', Ufdbk, self._callback_feedback_mode, queue_size=10)
        # rospy.Subscriber('compliant/record' , Record, self._callback_record, queue_size=10)
        # rospy.Subscriber('kinova/joint_states', JointState, self._callback_joints, queue_size=10)

        # # -- publishers --
        # self.pub_mode = rospy.Publisher("compliant/make_compliant_joint" , Bool, queue_size=1)
        # self.pub_desired_joints = rospy.Publisher("compliant/desired_joints" , JointState, queue_size=1)
        # self.pub_stiffness = rospy.Publisher("compliant/set_stiffness_joints" , Float32MultiArray, queue_size=1)
        
        
    def _callback_feedback_mode(self, data):
        self.mode = data.mode

    def _callback_joystick(self, data):
        if data.buttons[1] and self.end == False: #cross button
            self.end = True 
            print("stop compliant controller")
        if data.buttons[2] and self.start == False:
            self.start = True
            print("start compliant controller")
            
    def _callback_record(self, msg):
        self.target = msg.relative_target
        
    def _callback_desired_pose(self, msg):
        self.position_desired = [msg.position.x, msg.position.y, msg.position.z]
        self.quaternion_desired = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
            
    def publish_stiffness(self):
        stiffness_msg = Float32MultiArray()
        diag_stiffness_list = [self.Kd[i, i] for i in range(len(self.Kd))] + [self.Dd[i, i] for i in range(len(self.Dd))]
        print("diag_stiffness_list: ", diag_stiffness_list)
        stiffness_msg.data = diag_stiffness_list
        self.pub_stiffness.publish(stiffness_msg)
        
    def publish_desired_pose(self):
        desired_pose = [self.target[i] + self.desired_pose_offset[i] for i in range(3)]
        desired_pose_msg = Pose()
        desired_pose_msg.position.x = desired_pose[0]
        desired_pose_msg.position.y = desired_pose[1]
        desired_pose_msg.position.z = desired_pose[2]
        self.pub_desired_pose.publish(desired_pose_msg)
        
    def load_recording(self, file_name_recording):
        file_name = file_name_recording + ".pk"
        folder_path = "demonstrations"
        pickle_file_path = os.path.join(folder_path, file_name)
        with open(pickle_file_path, 'rb') as file:
            self.data_recording = pickle.load(file)
            self.q_d_list = self.data_recording["q"]
            self.pose_base = self.data_recording["base_pose"]

    def start_compliant_mode(self):
        print("Making the robot compliant, do not press any joystick-keys!!")
        mode = Bool()
        mode.data = True
        self.pub_mode.publish(mode)
        time.sleep(15)
        self.start = False
        # self.desired_pose = self.pose_current
        print("--- Compliant mode ready ---")
        print("To stop the compliant mode, press the circle-button on the joystick")
        print("Always stop the compliant mode, before doing ctrl+C")
        
    def stop_compliant_mode(self):
        mode = Bool()
        mode.data = False
        self.end = False
        self.pub_mode.publish(mode)
        #exit
        time.sleep(2)
        print(" --- Compliant mode stopped ---")
        print("To start the compliant mode, press the square-button on the joystick")
            
    def run(self):
        # publish pose goal
        # -- publish stiffness --
        if self.stiffness_enabled == False:
            self.publish_stiffness()
            self.stiffness_enabled = True
        
        # check if compliant mode is activated, otherwise active:
        if self.mode != "LLC_task" and self.start:
             self.start_compliant_mode()
            # print("Making the robot compliant, do not press any keys!!")
            # mode = Bool()
            # mode.data = True
            # self.pub_mode.publish(mode)
            # time.sleep(15)
            # print("You can press keys now!!!")
            
        if self.mode == "LLC_task" and self.TARGET_RESET == False: # and self.target != None:
            print("reached here in LLC task!!")
            self.publish_desired_pose()
            self.publish_stiffness()
            self.TARGET_RESET = False
        
        # if "X" button pressed on joystick, the compliant mode is deactivated:
        if self.end:
            self.stop_compliant_mode()
            # mode = Bool()
            # mode.data = False
            # self.pub_mode.publish(mode)
            # #exit
            # time.sleep(2)
            # exit()
        
if __name__ == '__main__':
    # rospy.init_node('tracker_compliant')
    state_recorder = TrackerCompliant() #sys.argv[1])
    rate = rospy.Rate(100)
    rospy.sleep(0.1)
    while not rospy.is_shutdown():
        try:
            state_recorder.run()
            rate.sleep()
        except rospy.ROSInterruptException:
            pass

    
    