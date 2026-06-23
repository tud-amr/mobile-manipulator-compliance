import numpy as np
import rospy
import sys
import os
import pickle
from geometry_msgs.msg import Pose
import time
"""
This script is only tested using the "normal" dinova controller (non-compliant controller) at the moment!!

has to be run beforehand:
    roslaunch dinova_bringup dinova.launch
    roslaunch dinova_fabrics_wrapper dinova_pose_no_actionsv.launch
"""

class PlaybackRecording:
    def __init__(self, robot_name, file_name_recording="recording_demonstration_0", topic_name = "kinova/command"):
        self.robot_name = robot_name
        self.file_name = file_name_recording
        self.iter = 0
        self.x_pos_list = []
        self.x_quat_list = []
        self.load_recording(file_name_recording)
        
        # self.pub_kinova_command = rospy.Publisher("%s/kinova_fabrics/joint_space_goal" % robot_name, Float64MultiArray, queue_size=1)
        self.pub_kinova_command = rospy.Publisher(str(robot_name)+"/"+str(topic_name), Pose, queue_size=1)
        
    def load_recording(self, file_name_recording):
        file_name = file_name_recording + ".pk"
        folder_path = "demonstrations"
        pickle_file_path = os.path.join(folder_path, file_name)
        with open(pickle_file_path, 'rb') as file:
            self.data_recording = pickle.load(file)
            self.x_pos_list = self.data_recording["pos_fk_downsample"]
            print("x_pos_list: ", self.x_pos_list)
            self.x_quat_list = self.data_recording["quat_fk_downsample"]
            
    def send_dinova_sequence(self, x_pos_desired, x_quat_desired=[0, 0, 0, 0]):
        x_desired_msg = Pose()
        x_desired_msg.position.x = x_pos_desired[0]
        x_desired_msg.position.y = x_pos_desired[1]
        x_desired_msg.position.z = x_pos_desired[2]
        
        x_desired_msg.orientation.x = x_quat_desired[0]
        x_desired_msg.orientation.y = x_quat_desired[1]
        x_desired_msg.orientation.z = x_quat_desired[2]
        x_desired_msg.orientation.w = x_quat_desired[3]
        print("x_pos_desired: ", x_pos_desired)
        print("x_orient_desired: ", x_quat_desired)
        self.pub_kinova_command.publish(x_desired_msg)
            
    def run(self):
        if len(self.x_pos_list)>0:
            if (len(self.x_pos_list) > self.iter):
                self.send_dinova_sequence(list(self.x_pos_list[self.iter]), list(self.x_quat_list[self.iter]))
                self.iter += 1
                print("self.iter: ", self.iter)
            else:
                print("sequence executed!!")
                exit()
        
if __name__ == '__main__':
    rospy.init_node('playback_recording')
    if len(sys.argv) > 2:
        arg2 = sys.argv[2]
    else:
        arg2 = "recording_demonstration_17"
    if len(sys.argv) > 3:
        arg3 = str(sys.argv[3])
    else:
        arg3 = "dinova_fabrics/pose_goal"#"kinovaaa/command"
    playback_recording = PlaybackRecording(sys.argv[1], file_name_recording=arg2, topic_name=arg3)
    rate = rospy.Rate(10)
    rospy.sleep(0.1)
    while not rospy.is_shutdown():
        try:
            playback_recording.run()
            rate.sleep()
        except rospy.ROSInterruptException:
            pass
        
        