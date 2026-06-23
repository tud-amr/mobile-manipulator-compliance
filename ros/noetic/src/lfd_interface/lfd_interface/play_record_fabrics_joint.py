import numpy as np
import rospy
import sys
import os
import pickle
from std_msgs.msg import Float64MultiArray
import time
"""
This script is only tested using the "normal" dinova controller (non-compliant controller) at the moment!!

has to be run beforehand:
    roslaunch dinova_bringup dinova.launch
    roslaunch dinova_fabrics_wrapper dinova_joint_space_no_actionsv.launch
"""

class PlaybackRecording:
    def __init__(self, robot_name, file_name_recording="recording_demonstration_0", topic_name = "kinova/command"):
        self.robot_name = robot_name
        self.file_name = file_name_recording
        self.iter = 0
        self.q_list = []
        self.load_recording(file_name_recording)
        
        # self.pub_kinova_command = rospy.Publisher("%s/kinova_fabrics/joint_space_goal" % robot_name, Float64MultiArray, queue_size=1)
        self.pub_kinova_command = rospy.Publisher(str(robot_name)+"/"+str(topic_name), Float64MultiArray, queue_size=1)
        
    def load_recording(self, file_name_recording):
        file_name = file_name_recording + ".pk"
        folder_path = "demonstrations"
        pickle_file_path = os.path.join(folder_path, file_name)
        with open(pickle_file_path, 'rb') as file:
            self.data_recording = pickle.load(file)
            self.q_list = self.data_recording["q"]
            self.pose_base = self.data_recording["base_pose"]
            
    def send_dinova_sequence(self, q_desired, base_desired=[0, 0, 0]):
        q_desired_msg = Float64MultiArray()
        q_desired_tot = base_desired + q_desired
        q_desired_msg.data = q_desired_tot
        print("q_desired: ", q_desired_tot)
        self.pub_kinova_command.publish(q_desired_msg)
        
    def send_dingo_sequence(self, base_desired):
        # print("base_desired: ", base_desired)
        kkkk=1
            
    def run(self):
        if len(self.q_list)>0:
            if (len(self.q_list) > self.iter):
                self.send_dinova_sequence(list(self.q_list[self.iter]), list(self.pose_base[self.iter]))
                # self.send_dingo_sequence(list(self.pose_base[self.iter]))
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
        arg2 = "recording_demonstration_4"
    if len(sys.argv) > 3:
        arg3 = str(sys.argv[3])
    else:
        arg3 = "dinova_fabrics/joint_space_goal"#"kinovaaa/command"
    playback_recording = PlaybackRecording(sys.argv[1], file_name_recording=arg2, topic_name=arg3)
    rate = rospy.Rate(100)
    rospy.sleep(0.1)
    while not rospy.is_shutdown():
        try:
            playback_recording.run()
            rate.sleep()
        except rospy.ROSInterruptException:
            pass
        
        