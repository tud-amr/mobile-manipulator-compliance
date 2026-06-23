import rospy
import time
import pickle
import sys
import os
import numpy as np
from user_interface_msg.msg import Record, Ufdbk, Record
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray, Bool
from analysis_utils import analysis_utils
from geometry_msgs.msg import Pose
import copy

"""
has to be run beforehand:
    roslaunch launcher robot_lfd_kinova.launch
"""

class TrackerCompliant():
    def __init__(self, robot_name, file_name_recording="recording_demonstration_1"):
        # -- parameters --#
        self.robot_name = robot_name
        self.Kd = np.eye(3) * 100
        self.Dd = np.eye(3) * 3
        self.desired_pose_offset = [0., 0., 0.]
        # self.Kd = np.eye(3) * 0.000000001
        # self.Dd = np.eye(3) * 0.000000001
        self.end = False
        self.TARGET_RESET = False
        self.target = None
        self.stiffness_enabled = False
        self.mode = "Unknown"
        
        # -- recording --
        self.file_name = file_name_recording
        self.iter = 0
        self.pos_d_list = []
        self.pos_current = [0]*6
        self.load_recording(file_name_recording)
        self.analysis = analysis_utils()
        
        # -- subscribers -- 
        rospy.Subscriber('%s/bluetooth_teleop/joy' % robot_name, Joy, self._callback_joystick, queue_size=10)
        rospy.Subscriber('%s/compliant/feedback' % robot_name, Ufdbk, self._callback_feedback_mode, queue_size=10)
        rospy.Subscriber('%s/compliant/record' % robot_name, Record, self._callback_record, queue_size=10)

        # -- publishers --
        self.pub_mode = rospy.Publisher("%s/compliant/make_compliant" % robot_name, Bool, queue_size=1)
        self.pub_desired_pose = rospy.Publisher("%s/compliant/desired_pose" % robot_name, Pose, queue_size=1)
        self.pub_stiffness = rospy.Publisher("%s/compliant/set_stiffness" % robot_name, Float32MultiArray, queue_size=1)
        
        
    def _callback_feedback_mode(self, data):
        self.mode = data.mode

    def _callback_joystick(self, data):
        if data.buttons[0] and self.end == False: #cross button
            self.end = True 
            print("stop compliant controller")
            
    def _callback_record(self, msg):
        self.target = msg.relative_target
        self.pos_current = msg.pos_x
            
    def publish_stiffness(self):
        stiffness_msg = Float32MultiArray()
        diag_stiffness_list = [self.Kd[i, i] for i in range(len(self.Kd))] + [self.Dd[i, i] for i in range(len(self.Dd))]
        print("diag_stiffness_list: ", diag_stiffness_list)
        stiffness_msg.data = diag_stiffness_list
        self.pub_stiffness.publish(stiffness_msg)
        
    def publish_desired_pose(self, desired_pose=None):
        if desired_pose == None:
            desired_pose = [self.target[i] + self.desired_pose_offset[i] for i in range(3)]
        desired_pose_msg = Pose()
        desired_pose_msg.position.x = desired_pose[0]
        desired_pose_msg.position.y = desired_pose[1]
        desired_pose_msg.position.z = desired_pose[2]
        print("desired_pose: ", desired_pose)
        self.pub_desired_pose.publish(desired_pose_msg)
        
    def load_recording(self, file_name_recording):
        file_name = file_name_recording + ".pk"
        folder_path = "demonstrations"
        pickle_file_path = os.path.join(folder_path, file_name)
        with open(pickle_file_path, 'rb') as file:
            self.data_recording = pickle.load(file)
            self.pos_d_list = self.data_recording["ee_position"]
            self.pose_base = [0., 0., 0.] #self.data_recording["base_pose"]
            
    def run(self):
        # publish pose goal
        # -- publish stiffness --
        if self.stiffness_enabled == False:
            self.publish_stiffness()
            self.stiffness_enabled = True
        
        # check if compliant mode is activated, otherwise active:
        if self.mode != "LLC_task":
            print("Making the robot compliant, do not press any keys!!")
            mode = Bool()
            mode.data = True
            self.pub_mode.publish(mode)
            time.sleep(15)
            print("You can press keys now!!!")
            
        # if self.mode == "LLC_task" and self.TARGET_RESET == False and self.target != None:
        #     self.publish_desired_pose()
        #     self.TARGET_RESET = True
            
        if len(self.pos_d_list)>0:
            if (len(self.pos_d_list) > self.iter):
                self.publish_desired_pose(list(self.pos_d_list[self.iter]))
                self.analysis.record_deviation_recording(np.array(self.pos_current), np.array(self.pos_d_list[self.iter]))
                self.iter += 1
                print("self.iter: ", self.iter)
            else:
                self.analysis.return_average_deviation()
                print("sequence executed!!")
                mode = Bool()
                mode.data = False
                self.pub_mode.publish(mode)
                time.sleep(2)
                exit()
        
        # if "X" button pressed on joystick, the compliant mode is deactivated:
        if self.end:
            mode = Bool()
            mode.data = False
            self.pub_mode.publish(mode)
            time.sleep(2)
            exit()
        
if __name__ == '__main__':
    rospy.init_node('tracker_compliant')
    if len(sys.argv) > 2:
        arg2 = sys.argv[2]
    else:
        arg2 = "recording_demonstration_13"
    state_recorder = TrackerCompliant(sys.argv[1], file_name_recording=arg2)
    rate = rospy.Rate(30)
    rospy.sleep(0.1)
    while not rospy.is_shutdown():
        try:
            state_recorder.run()
            rate.sleep()
        except rospy.ROSInterruptException:
            pass

    
    