import numpy as np
import rospy
import sys
import os
import pickle
from std_msgs.msg import Float64MultiArray
import time
from analysis_utils import analysis_utils
from sensor_msgs.msg import JointState
"""
This script is only tested using the "normal" dinova controller (non-compliant controller) at the moment!!

has to be run beforehand:
    roslaunch dinova_bringup dinova.launch
    rosservice call /dingo2/kinova/change_to_HLC_position
"""

class PlaybackRecording:
    def __init__(self, robot_name, file_name_recording="recording_demonstration_0"):
        self.robot_name = robot_name
        self.file_name = file_name_recording
        self.iter = 0
        self.q_d_list = []
        self.q_current = [0]*6
        self.load_recording(file_name_recording)
        self.analysis = analysis_utils()
        
        self.pub_kinova_command = rospy.Publisher("%s/kinova/command" % robot_name, Float64MultiArray, queue_size=1)
        rospy.Subscriber("%s/kinova/joint_states" % robot_name, JointState, self.read_q_current, queue_size=10)
        #self.pub_kinova_command_test = rospy.Publisher("%s/kinovaaa/command" % robot_name, Float64MultiArray, queue_size=1)
        # change from velocity control mode to position control mode:
        # service_name = '/%s/kinova/change_to_HLC_position' % robot_name
        # rospy.wait_for_service(service_name)
        # rospy.sleep(1)
        print("!!!Have you switched to HLC position mode before doing this, otherwise stop this script!!!")
        time.sleep(1)
        
    def load_recording(self, file_name_recording):
        file_name = file_name_recording + ".pk"
        folder_path = "demonstrations"
        pickle_file_path = os.path.join(folder_path, file_name)
        with open(pickle_file_path, 'rb') as file:
            self.data_recording = pickle.load(file)
            self.q_d_list = self.data_recording["q"]
            self.pose_base = self.data_recording["base_pose"]
            
    def send_kinova_sequence(self, q_desired):
        q_desired_msg = Float64MultiArray()
        q_desired_msg.data = q_desired
        self.pub_kinova_command.publish(q_desired_msg)
        
    def read_q_current(self, msg: JointState):
        q_current_msg = msg
        self.q_current = q_current_msg.position[0:6]
            
    def run(self):
        if len(self.q_d_list)>0:
            if (len(self.q_d_list) > self.iter):
                self.send_kinova_sequence(list(self.q_d_list[self.iter]))
                self.analysis.record_deviation_recording(np.array(self.q_current), np.array(self.q_d_list[self.iter]))
                self.iter += 1
                print("self.iter: ", self.iter)
            else:
                self.analysis.return_average_deviation()
                print("sequence executed!!")
                exit()
        
if __name__ == '__main__':
    rospy.init_node('playback_recording')
    if len(sys.argv) > 2:
        arg2 = sys.argv[2]
    else:
        arg2 = "recording_demonstration_11"
    playback_recording = PlaybackRecording(sys.argv[1], file_name_recording=arg2)
    rate = rospy.Rate(50)
    rospy.sleep(0.1)
    while not rospy.is_shutdown():
        try:
            playback_recording.run()
            rate.sleep()
        except rospy.ROSInterruptException:
            pass
        
        