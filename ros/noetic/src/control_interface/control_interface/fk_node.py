import rospkg
import rospy
from forwardkinematics.urdfFks.generic_urdf_fk import GenericURDFFk
from dinova_control.dinova_fk import FK_Autogen
import numpy as np
from sensor_msgs.msg import JointState
from typing import Union, Dict, List
import copy
from derived_object_msgs.msg import Object, ObjectArray
from geometry_msgs.msg import PoseStamped
from compliant_control.control.state import rotMatrix_to_quaternion

class FKNode():
    def __init__(self): #, lib_name=None, robot_type="dinova"):
        robot_name = "dingo2"
        lib_name = rospy.get_param('fk_library')
        self.robot_type = rospy.get_param("robot_type")
        rospy.init_node("fk_node")
        self._rate = rospy.Rate(100)
        
        # ---- variables from yaml file ---- #
        self.robot_name = robot_name
        self.lidar = rospy.get_param('lidar', False)
        
        # ---------------------------------------- #
        rospack = rospkg.RosPack()
        if self.lidar:
            agent_name = "dinova_lidar"
        else:
            agent_name = "dinova"
        # URDF_FILE = rospack.get_path("dinova_fabrics_wrapper") + "/config/" + agent_name + ".urdf"
        # self.symbolic_fk(URDF_FILE)
        
        # --- subscriber ---#
        if self.robot_type == "dinova":
            self.dof = 9
            rospy.Subscriber("dinova/omni_states_vicon", JointState, self._joint_states_cb, queue_size=10)
        else:
            rospy.Subscriber("kinova/joint_states", JointState, self._joint_states_cb, queue_size=10)
            self.dof = 6
        self._q = [0]*9
        self._q_kinova = [0]*self.dof
        
        # --- publisher ---#
        # self.pub_current_pose = rospy.Publisher("compliant/fk/current_pose", PoseStamped, queue_size=1)
        
        # forward kinematics publishing
        if self.robot_type == "kinova":
            self._robot_fk_autogen = FK_Autogen(lib_name)
            self._end_link  = self._robot_fk_autogen.get_endeffector_name()
            self.pub_robot_fk = rospy.Publisher('kinova/fk_links', ObjectArray, queue_size=1)
            self.pub_robot_endeffector_fk = rospy.Publisher('kinova/fk_endeffector', PoseStamped, queue_size=1)
        # print("reached here!!")

    def _joint_states_cb(self, msg):
        self._q[-self.dof:] = np.array(msg.position)[-self.dof:]
        self._q_kinova = self._q[-6:]
        
    def publish_pose(self, ee_position, ee_orientation=None):
        list_position = list(ee_position)
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = list_position[0]
        pose.pose.position.y = list_position[1]
        pose.pose.position.z = list_position[2]
        if ee_orientation is not None:  
            list_orientation = list(ee_orientation)
            pose.pose.orientation.x = list_orientation[0]
            pose.pose.orientation.y = list_orientation[1]
            pose.pose.orientation.z = list_orientation[2]
            pose.pose.orientation.w = list_orientation[3]
        self.pub_current_pose.publish(pose)
        
    def publish_FK_endeffector(self, pose_W_dict: dict):
        pose_W_EEF = pose_W_dict[self._end_link]
        endeffector_pose = PoseStamped()
        endeffector_pose.header.frame_id = self._end_link
        endeffector_pose.pose = pose_W_EEF
        
        self.pub_robot_endeffector_fk.publish(endeffector_pose)

    def publish_FK_links(self, pose_W_dict: dict):
        object_array = ObjectArray()
        object_array.header.stamp = rospy.Time.now()
        for link_name, transf in pose_W_dict.items():
            obj = Object()
            obj.header = copy.deepcopy(object_array.header)
            obj.header.frame_id = link_name
            obj.pose = transf
            object_array.objects.append(obj)
        self.pub_robot_fk.publish(object_array)     
        
    # def symbolic_fk(self, URDF_FILE) -> GenericURDFFk:
    #     with open(URDF_FILE, "r", encoding="utf-8") as file:
    #         urdf = file.read()
    #     self.forward_kinematics = GenericURDFFk(
    #         urdf,
    #         root_link="base_link",
    #         end_links=["arm_tool_frame"],
    #     )
        
    # def fk_numerical(self):
    #     ee_T = self.forward_kinematics.numpy(q=self._q,
    #                                             parent_link = "base_link",
    #                                             child_link = "arm_end_effector_link",
    #                                             position_only=False)
    #     ee_position = ee_T[:3, 3]
    #     ee_quaternion = rotMatrix_to_quaternion(ee_T[:3, :3])
    #     return ee_position, ee_quaternion
    
    def run(self):
        while not rospy.is_shutdown():
            """
            if 'mug1' in self._object_poses:
                self._logger.log(str(self._object_poses['mug1']))
            """
            # ee_position, ee_orientation = self.fk_numerical()
            # self.publish_pose(ee_position, ee_orientation)
            
            # poses via forward kinematics published:
            if self.robot_type == "kinova":
                q_act = np.asarray(copy.deepcopy(self._q_kinova))
                pose_W_dict = self._robot_fk_autogen.compute_fk(q_act)
                self.publish_FK_endeffector(pose_W_dict=pose_W_dict)
                self.publish_FK_links(pose_W_dict=pose_W_dict)
         
         
            self._rate.sleep()
    
if __name__ == "__main__":
    node = FKNode() #lib_name=rospy.get_param('fk_library'), robot_type=rospy.get_param("robot_type"))
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass