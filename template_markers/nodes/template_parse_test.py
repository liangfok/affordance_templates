#!/usr/bin/env python
import yaml
import argparse
import copy
import PyKDL as kdl

import rospy
import rospkg
import roslib; roslib.load_manifest("template_markers")

import geometry_msgs.msg
import sensor_msgs.msg
import visualization_msgs.msg

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *

from template_markers.template_utilities import *
import template_markers.atdf_parser

from nasa_robot_teleop.moveit_interface import *
# from nasa_robot_teleop.marker_helper import *
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
                # print self.end_effector_markers[g]

        # what do we have?
        # self.moveit_interface.print_basic_info()
        return True

    def print_yaml(self) :
        if not self.yaml_config == None:
            print "============================="
            print "Robot Config Info: "
            print "============================="
            print " robot name: ", self.yaml_config['robot_name']
            print " moveit_config: ", self.yaml_config['moveit_config_package']
            for ee in self.yaml_config['end_effector_map']:
                print "\t", "map: ", ee['name'], " --> ", ee['id']
                print "\t", "pose: ", ee['pose_offset']
            print "============================="

    def joint_state_callback(self, data) :
        self.joint_data = data

class AffordanceTemplate(object) :

    def __init__(self, server, id, name, initial_pose=None, robot_config=None):
        self.menu_handler = MenuHandler()
        self.menu_handler.insert("Delete", callback=self.delete_callback)
        self.server = server
        self.frame_id = "world"
        self.key = name + str(id)
        self.root_object = ""
        self.parent_map = {}
        self.marker_map = {}
        self.callback_map = {}
        self.marker_pose_offset = {}

        # not all templates will have a dynamic reconfigure server
        self.dserver = None
        self.initial_pose = initial_pose

        if not isinstance(robot_config, RobotConfig) :
            rospy.loginfo("AffordanceTemplate::init() -- problem setting robot config")
        else :
            self.robot_config = robot_config

    def set_root_object(self, name) :
        self.root_object = name

    def get_root_object(self) :
        return self.root_object

    def add_interactive_marker(self, marker, callback=None):
        name = marker.name
        self.marker_map[name] = marker
        if callback:
            self.callback_map[name] = callback
            return self.server.insert(marker, callback)
        else:
            self.callback_map[name] = self.process_feedback
            return self.server.insert(marker, self.process_feedback)

    def remove_interactive_marker(self, marker_name):
        if self.server.erase(marker_name):
            if marker_name in self.marker_map:
                del self.marker_map[marker_name]
                del self.callback_map[marker_name]
                return True
        return False

    def attach_menu_handler(self, marker):
        return self.menu_handler.apply(self.server, marker.name)

    def get_marker(self):
        return self.server.get(self._key)

    def has_marker(self):
        if self._key in self.marker_map.keys():
            return True
        else:
            return False

    def delete_callback(self, feedback):
        for key in self.marker_map.keys():
            self.server.erase(key)
        self.server.applyChanges()
        self.tear_down()

    def tear_down(self, keep_alive=False):
        # forcefully shut down service before killing node
        if self.dserver:
            self.dserver.set_service.shutdown("User deleted template.")
        # for rostest (and potentially other cases), we want to clean up but keep the node alive
        if not keep_alive:
            rospy.signal_shutdown("User deleted template.")

    def process_feedback(self, feedback):
        # print feedback.marker_name
        for m in self.marker_map.keys() :
            if m in self.parent_map :
                if self.parent_map[m] == feedback.marker_name :
                    rTm = getFrameFromPose(feedback.pose)
                    T = getFrameFromPose(self.marker_pose_offset[m])
                    p = getPoseFromFrame(rTm*T)
                    self.server.setPose(m, p)
        self.server.applyChanges()
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP :
            To = getFrameFromPose(self.marker_pose_offset[feedback.marker_name])
            T = getFrameFromPose(feedback.pose)

            if feedback.marker_name in self.parent_map:
                Tp = getFrameFromPose(self.marker_pose_offset[self.parent_map[feedback.marker_name]])
                self.marker_pose_offset[feedback.marker_name] = getPoseFromFrame(Tp.Inverse()*T)
            else :
                self.marker_pose_offset[feedback.marker_name] = getPoseFromFrame(T)


class AffordanceTemplateCreator(object) :
    def __init__(self,server,robot_config=None) :
        self.structure = None
        self.affordamce_template = None
        self.robot_config = robot_config
        self.server = server

        self.end_effector_link_data = {}

    def pose_from_origin(self, origin) :
        p = geometry_msgs.msg.Pose()
        p.orientation.w = 1
        try:
            q = (kdl.Rotation.RPY(origin.rpy[0],origin.rpy[1],origin.rpy[2])).GetQuaternion()
            p.position.x = origin.xyz[0]
            p.position.y = origin.xyz[1]
            p.position.z = origin.xyz[2]
            p.orientation.x = q[0]
            p.orientation.y = q[1]
            p.orientation.z = q[2]
            p.orientation.w = q[3]
        except :
            rospy.logerr("AffordanceTemplateCreator::pose_from_origin() error")
        return p

    def create_from_structure(self) :

        if self.structure == None :
            return False

        self.affordance_template = AffordanceTemplate(self.server, 0, self.structure.name, robot_config=self.robot_config)

        # parse objects
        ids = 0
        for obj in self.structure.display_objects.display_objects :

            if ids == 0:
                self.affordance_template.set_root_object(obj.name)

            p = self.pose_from_origin(obj.origin)

            int_marker = InteractiveMarker()
            control = InteractiveMarkerControl()

            int_marker.header.frame_id = self.affordance_template.frame_id
            int_marker.pose = p
            int_marker.name = obj.name
            int_marker.description = obj.name
            int_marker.scale = obj.controls.scale

            if isinstance(obj.geometry, template_markers.atdf_parser.Mesh) :
                control = CreatePrimitiveControl(obj.name, p, 1.0, Marker.MESH_RESOURCE, ids)
                control.markers[0].mesh_resource = obj.geometry.filename
                control.markers[0].mesh_use_embedded_materials = True
            elif isinstance(obj.geometry, template_markers.atdf_parser.Box) :
                control = CreatePrimitiveControl(obj.name, p, 1.0, Marker.CUBE, ids)
                control.markers[0].scale.x = obj.geometry.size[0]
                control.markers[0].scale.y = obj.geometry.size[1]
                control.markers[0].scale.z = obj.geometry.size[2]
            elif isinstance(obj.geometry, template_markers.atdf_parser.Sphere) :
                control = CreatePrimitiveControl(obj.name, p, 1.0, Marker.SPHERE, ids)
                control.markers[0].scale.x = obj.geometry.radius
                control.markers[0].scale.y = obj.geometry.radius
                control.markers[0].scale.z = obj.geometry.radius
            elif isinstance(obj.geometry, template_markers.atdf_parser.Cylinder) :
                control = CreatePrimitiveControl(obj.name, p, 1.0, Marker.CYLINDER, ids)
                control.markers[0].scale.x = obj.geometry.radius
                control.markers[0].scale.y = obj.geometry.radius
                control.markers[0].scale.z = obj.geometry.length

            if not obj.material == None :
                control.markers[0].color.r = obj.material.color.rgba[0]
                control.markers[0].color.g = obj.material.color.rgba[1]
                control.markers[0].color.b = obj.material.color.rgba[2]
                control.markers[0].color.a = obj.material.color.rgba[3]
                if isinstance(obj.geometry, template_markers.atdf_parser.Mesh) :
                    control.markers[0].mesh_use_embedded_materials = False

            control.markers[0].header.frame_id = self.affordance_template.frame_id
            control.markers[0].action = Marker.ADD
            # print control.markers[0]

            scale = 1.0
            if obj.controls.scale != None :
                scale = obj.controls.scale

            # int_marker = CreateInteractiveMarker(self.affordance_template.frame_id, obj.name, scale)
            int_marker.controls.append(control)
            int_marker.controls.extend(CreateCustomDOFControls("",
                obj.controls.xyz[0], obj.controls.xyz[1], obj.controls.xyz[2],
                obj.controls.rpy[0], obj.controls.rpy[1], obj.controls.rpy[2]))

            self.affordance_template.add_interactive_marker(int_marker)
            self.server.applyChanges()

            self.affordance_template.marker_map[obj.name] = control.markers[0]
            self.affordance_template.marker_pose_offset[obj.name] = p

            if not obj.parent == None :
                self.affordance_template.parent_map[obj.name] = obj.parent

            ids += 1

        # parse end effector information
        wp_ids = 0
        for wp in self.structure.end_effector_waypoints.end_effector_waypoints :

            wp_name = str(wp.end_effector) + "." + str(wp.id)
            ee_name = self.robot_config.end_effector_name_map[int(wp.end_effector)]

            p = self.pose_from_origin(wp.origin)
            root_pose = self.affordance_template.marker_pose_offset[wp.display_object]
            ee_pose = self.robot_config.end_effector_pose_map[ee_name]
            display_pose = getPoseFromFrame(getFrameFromPose(root_pose)*getFrameFromPose(p)*getFrameFromPose(ee_pose))
            int_marker = InteractiveMarker()
            control = InteractiveMarkerControl()

            int_marker.header.frame_id = self.affordance_template.frame_id
            int_marker.pose = display_pose
            int_marker.name = wp_name
            int_marker.description = wp_name
            int_marker.scale = wp.controls.scale

            menu_control = InteractiveMarkerControl()
            menu_control.interaction_mode = InteractiveMarkerControl.BUTTON

            for m in self.robot_config.end_effector_markers[ee_name].markers :
                ee_m = copy.deepcopy(m)
                ee_m.header.frame_id = self.affordance_template.frame_id
                ee_m.pose = getPoseFromFrame(getFrameFromPose(display_pose)*getFrameFromPose(m.pose))
                ee_m.color.r = .2
                ee_m.color.g = .5
                ee_m.color.b = .2
                ee_m.color.a = .8
                menu_control.markers.append( ee_m )

            scale = 1.0
            if wp.controls.scale != None :
                scale = wp.controls.scale

            int_marker.controls.append(menu_control)
            int_marker.controls.extend(CreateCustomDOFControls("",
                wp.controls.xyz[0], wp.controls.xyz[1], wp.controls.xyz[2],
                wp.controls.rpy[0], wp.controls.rpy[1], wp.controls.rpy[2]))

            self.affordance_template.add_interactive_marker(int_marker)
            self.server.applyChanges()

            self.affordance_template.marker_pose_offset[wp_name] = p

            if not wp.display_object == None :
                 self.affordance_template.parent_map[wp_name] = wp.display_object
            else :
                self.affordance_template.parent_map[wp_name] = self.affordance_template.get_root_object()
            wp_ids += 1



    def load_from_file(self, filename) :
        self.structure = template_markers.atdf_parser.AffordanceTemplateStructure.from_file(filename)
        self.create_from_structure()
        return self.structure

    def print_structure(self) :
        print "---------------------"
        print "---------------------"
        print "found new template:"
        print "  name:  ", self.structure.name
        print "  image: ", self.structure.image
        for obj in self.structure.display_objects.display_objects :
            print "---------------------"
            print "\tobject: ", obj.name
            if not obj.parent == None :
                print "\tparent: ", obj.parent

            print "----------------"
            print "\torigin xyz: ", obj.origin.xyz
            print "\torigin rpy: ", obj.origin.rpy
            print "----------------"
            print "\tcontrols xyz: ", obj.controls.xyz
            print "\tcontrols rpy: ", obj.controls.rpy
            print "----------------"
            if isinstance(obj.geometry, template_markers.atdf_parser.Box) :
                print "\tBox size: ", obj.geometry.size
            if isinstance(obj.geometry, template_markers.atdf_parser.Sphere) :
                print "\tSphere radius: ", obj.geometry.radius
            if isinstance(obj.geometry, template_markers.atdf_parser.Cylinder) :
                print "\tCylinder size: (", obj.geometry.radius, ", ", obj.geometry.length, ")"
            if isinstance(obj.geometry, template_markers.atdf_parser.Mesh) :
                print "\tMesh: ", obj.geometry.filename
            if not obj.material == None :
                print "\tmaterial: ", obj.material.name

        print "---------------------"
        for wp in self.structure.end_effector_waypoints.end_effector_waypoints :
            print "---------------------"
            print "\twaypoint: ", wp.end_effector, ".", wp.id
            print "----------------"
            print "\tdisplay_object: ", wp.display_object
            print "----------------"
            print "\torigin xyz: ", wp.origin.xyz
            print "\torigin rpy: ", wp.origin.rpy
            print "----------------"
            print "\tcontrols xyz: ", wp.controls.xyz
            print "\tcontrols rpy: ", wp.controls.rpy
            print "----------------"

if __name__ == '__main__':

    rospy.init_node('template_parse_test')

    rospack = rospkg.RosPack()
    atl_path = rospack.get_path('affordance_template_library')
    tm_path = rospack.get_path('template_markers')

    server = InteractiveMarkerServer("affordance_template_server")

    template_filename = atl_path + "/templates/test.atdf"
    robot_config_filename = tm_path + "/resources/robots/r2.yaml"

    try:

        # load RobotConfig from yaml and moveit package
        rc = RobotConfig()
        rc.load_from_file(robot_config_filename)

        # load in Affordance Template from file
        ats = AffordanceTemplateCreator(server, rc)
        at = ats.load_from_file(template_filename)
        # ats.print_structure()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass




