import numpy as np

class analysis_utils():
    def __init__(self):
        self.dist_recording_list = []
    
    def record_deviation_recording(self, q_current, q_record):
        dist_recording_q = np.mean(np.absolute(q_current-q_record))
        self.dist_recording_list.append(dist_recording_q)
        
    def return_average_deviation(self):
        print("The mean distance in joint space is: ", np.mean(self.dist_recording_list))
        return np.mean(self.dist_recording_list)
