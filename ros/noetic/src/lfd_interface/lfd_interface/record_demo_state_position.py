import rospy
import time
import pickle
import sys
import os
from user_interface_msg.msg import Record, Ufdbk
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from scipy.signal import butter, filtfilt, decimate

"""
A script to record the state (joint state + end-effector pose) of the robot.
Use the buttons on the dinova controller to start and stop a recording:
- triangle button: start recording
- cross button: stop recording
"""

class StateRecorder:
    def __init__(self, robot_name, save_id):
        self.save_id = save_id
        self.start = False
        self.end = False
        self.mode = "Unknown"
        self.q, self.q_dot, self.time_prev = None, None, None
        self.x_pos, self.x_orient = None, None
        self.pos_fk, self.quat_fk = None, None
        self.q_history, self.q_dot_history, self.time_history= [], [], []
        self.x_pos_history, self.x_quat_history, self.base_pose_history= [], [], []
        self.relative_target_history, self.absolute_target_history = [], []
        self.pos_fk_history, self.quat_fk_history = [], []
        self.fk_endeffector_pos, self.fk_endeffector_quat = None, None
        self.fk_endeffector_pos_history, self.fk_endeffector_quat_history = [], []
        self.joystick_data = None
        robot_type = rospy.get_param("%s/robot_type" % robot_name)

        #Create ROS subscriber
        rospy.Subscriber('%s/compliant/record' % robot_name, Record, self._callback_state, queue_size=10)
        rospy.Subscriber('%s/%s/fk_endeffector' % (robot_name, robot_type), PoseStamped, self._callback_fk_endeffector, queue_size=10)
        rospy.Subscriber('%s/compliant/feedback' % robot_name, Ufdbk, self._callback_feedback_mode, queue_size=10)
        rospy.Subscriber('%s/bluetooth_teleop/joy' % robot_name, Joy, self._callback_joystick, queue_size=10)
        self.pub_mode = rospy.Publisher("%s/compliant/make_compliant" % robot_name, Bool, queue_size=1)
        
    def _callback_state(self, data):
        self.q = data.pos_q
        self.q_dot = data.vel_q
        self.x_pos = data.pos_x
        self.x_quat = data.quat_x
        self.base_pose = data.pose_b
        self.time = data.time[0]
        self.relative_target = data.relative_target
        self.absolute_target = data.absolute_target
        self.pos_fk = data.pos_fk
        self.quat_fk = data.quat_fk
        
    def _callback_fk_endeffector(self, data):
        self.fk_endeffector_pos = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        self.fk_endeffector_quat = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
        
    def _callback_feedback_mode(self, data):
        self.mode = data.mode
    
    def _callback_joystick(self, data):
        if data.buttons[3] and self.start == False: #triangle button
            self.start = True
            print("recording started")
        if data.buttons[0] and self.end == False: #cross button
            self.end = True 
            print("recording ended")
    
    def _append_state(self):
        self.q_history.append(self.q)
        self.q_dot_history.append(self.q_dot)
        self.x_pos_history.append(self.x_pos)
        self.x_quat_history.append(self.x_quat)
        self.pos_fk_history.append(self.pos_fk)
        self.quat_fk_history.append(self.quat_fk)
        self.fk_endeffector_pos_history.append(self.fk_endeffector_pos)
        self.fk_endeffector_quat_history.append(self.fk_endeffector_quat)
        self.base_pose_history.append(self.base_pose)
        self.time_history.append(self.time)
        self.relative_target_history.append(self.relative_target)
        self.absolute_target_history.append(self.absolute_target)
        
    # def low_pass_filter(self, data, cutoff, fs, order=4):
    #     print("len data: ", len(data))
    #     nyquist = 0.5 * fs
    #     normal_cutoff = cutoff / nyquist
    #     b, a = butter(order, normal_cutoff, btype='low', analog=False)
    #     return filtfilt(b, a, data)
    
    def filter_data(self, position_noisy, orientation_noisy, fs=100, cutoff_frequency=5):
        # position_filtered = self.low_pass_filter(position_noisy, cutoff_frequency, fs)
        # orientation_filtered = self.low_pass_filter(orientation_noisy, cutoff_frequency, fs)
        
        downsample_factor = 10  # From 100 Hz to 10 Hz
        position_smooth = position_noisy[::downsample_factor]
        orientation_smooth = orientation_noisy[::downsample_factor] #decimate(orientation_noisy, downsample_factor, ftype='fir')
        return position_smooth, orientation_smooth

        
    def _save_trajectory(self, frequency=100):
        #Create dictionary
        trajectory = {"q": self.q_history,
                      "q_dot": self.q_dot_history,
                      "ee_position": self.x_pos_history,
                      "ee_quaternion": self.x_quat_history,
                      "pos_fk": self.pos_fk_history,
                      "quat_fk": self.quat_fk_history,
                      "fk_endeffector_pos": self.fk_endeffector_pos_history,
                      "fk_endeffector_quat": self.fk_endeffector_quat_history,
                      "base_pose": self.base_pose_history,
                      "t": self.time_history,
                      "relative_target": self.relative_target_history,
                      "absolute_target": self.absolute_target_history}
        #Save dictionary
        file_name = "recording_demonstration_" + self.save_id + ".pk"
        folder_path = "demonstrations"
        file_path = os.path.join(folder_path, file_name)
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)
            print(f"Folder '{folder_path}' created.")
        with open(file_path, 'wb') as file:
            pickle.dump(trajectory, file)
            
        # also save downsampled trajectory
        position_downsample, orientation_downsample = self.filter_data(position_noisy=self.pos_fk_history, 
                                                    orientation_noisy=self.quat_fk_history, 
                                                    fs=frequency)
        trajectory["pos_fk_downsample"] = position_downsample
        trajectory["quat_fk_downsample"] = orientation_downsample
        with open(file_path, 'wb') as file:
            pickle.dump(trajectory, file)
            
    def run(self, frequency=100):
        # check if compliant mode is activated, otherwise active:
        if self.mode != "LLC_task":
            print("Making the robot compliant, do not press any keys!!")
            mode = Bool()
            mode.data = True
            self.pub_mode.publish(mode)
            time.sleep(15)
            print("You can press keys now!!!")
        
        if self.start:
            #Append state to trajectory
            self._append_state()
            
        if self.end:
            mode = Bool()
            mode.data = False
            self.pub_mode.publish(mode)
            #Save trajectory and exit
            self._save_trajectory(frequency=frequency)
            time.sleep(2)
            exit()
        
    
if __name__ == '__main__':
    rospy.init_node('state_recorder')
    state_recorder = StateRecorder(sys.argv[1], save_id=sys.argv[2])
    frequency = 100
    rate = rospy.Rate(frequency)
    rospy.sleep(0.1)
    while not rospy.is_shutdown():
        try:
            state_recorder.run(frequency)
            rate.sleep()
        except rospy.ROSInterruptException:
            pass


    