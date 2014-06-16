
import yaml

import rospy
import tf
import PyKDL as kdl

import geometry_msgs.msg
import sensor_msgs.msg

from nasa_robot_teleop.moveit_interface import *
from nasa_robot_teleop.kdl_posemath import *
from nasa_robot_teleop.pose_update_thread import *
from nasa_robot_teleop.end_effector_helper import *

class RobotConfig(object) :
    def __init__(self) :

        self.robot_name = ""
        self.config_package =  ""

        self.moveit_ee_groups = []
        self.end_effector_names = []
        self.end_effector_id_map = {}
        self.end_effector_name_map = {}
        self.end_effector_pose_map = {}
        self.end_effector_link_data = {}
        self.end_effector_markers = {}
        self.frame_id = "world"
        self.root_offset = geometry_msgs.msg.Pose()
        self.tf_listener = tf.TransformListener()
        self.joint_data = sensor_msgs.msg.JointState()

        rospy.Subscriber("/joint_states", sensor_msgs.msg.JointState, self.joint_state_callback)

    def load_from_file(self, filename) :
        try:
            f = open(filename)
            self.yaml_config = yaml.load(f.read())
            f.close()
            self.print_yaml()

            self.robot_name = self.yaml_config['robot_name']
            self.config_package = str(self.yaml_config['moveit_config_package'])
            self.frame_id = self.yaml_config['frame_id']

            q = (kdl.Rotation.RPY(self.yaml_config['root_offset'][3],self.yaml_config['root_offset'][4],self.yaml_config['root_offset'][5])).GetQuaternion()
            self.root_offset.position.x = self.yaml_config['root_offset'][0]
            self.root_offset.position.y = self.yaml_config['root_offset'][1]
            self.root_offset.position.z = self.yaml_config['root_offset'][2]
            self.root_offset.orientation.x = q[0]
            self.root_offset.orientation.y = q[1]
            self.root_offset.orientation.z = q[2]
            self.root_offset.orientation.w = q[3]

            print "Root Offset:"
            print self.root_offset

            for ee in self.yaml_config['end_effector_map']:
                self.end_effector_names.append(ee['name'])
                self.end_effector_name_map[ee['id']] = ee['name']
                self.end_effector_id_map[ee['name']] = ee['id']
                p = geometry_msgs.msg.Pose()
                q = (kdl.Rotation.RPY(ee['pose_offset'][3],ee['pose_offset'][4],ee['pose_offset'][5])).GetQuaternion()
                p.position.x = ee['pose_offset'][0]
                p.position.y = ee['pose_offset'][1]
                p.position.z = ee['pose_offset'][2]
                p.orientation.x = q[0]
                p.orientation.y = q[1]
                p.orientation.z = q[2]
                p.orientation.w = q[3]
                self.end_effector_pose_map[ee['name']] = p
            return self.configure()

        except :
            rospy.logerr("RobotConfig::load_from_file() -- error opening config file")
            return False

    def configure(self) :
        self.moveit_interface = MoveItInterface(self.robot_name,self.config_package)
        self.root_frame = self.moveit_interface.get_planning_frame()
        self.moveit_ee_groups = self.moveit_interface.srdf_model.get_end_effector_groups()
        for g in self.end_effector_names :
            if not g in self.moveit_ee_groups:
                rospy.logerr("RobotConfig::configure() -- group ", g, " not in moveit end effector groups!")
                return False
            else :
                self.end_effector_link_data[g] = EndEffectorHelper(self.robot_name, g, self.moveit_interface.srdf_model.group_end_effectors[g].parent_link, self.tf_listener)
                self.end_effector_link_data[g].populate_data(self.moveit_interface.get_group_links(g), self.moveit_interface.get_urdf_model())
                print "got end effector markers for: ", g
                print "control frame: ", self.moveit_interface.srdf_model.group_end_effectors[g].parent_link
                rospy.sleep(2)
                self.end_effector_markers[g] = self.end_effector_link_data[g].get_current_position_marker_array(scale=1.0,color=(1,1,1,0.5))

        # what do we have?
        # self.moveit_interface.print_basic_info()
        return True

    def print_yaml(self) :
        if not self.yaml_config == None:
            print "============================="
            print "Robot Config Info: "
            print "============================="
            print " robot name: ", self.yaml_config['robot_name']
            print " root offset: ", self.yaml_config['root_offset']
            print " moveit_config: ", self.yaml_config['moveit_config_package']
            for ee in self.yaml_config['end_effector_map']:
                print "\t", "map: ", ee['name'], " --> ", ee['id']
                print "\t", "pose: ", ee['pose_offset']
            print "============================="

    def joint_state_callback(self, data) :
        self.joint_data = data
