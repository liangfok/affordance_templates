import yaml
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

from nasa_robot_teleop.moveit_interface import *

from template_markers.robot_config import *
from template_markers.template_utilities import *
import template_markers.atdf_parser

class AffordanceTemplate(object) :

    def __init__(self, server, id, name="affordance_template", initial_pose=None, robot_config=None):
        self.menu_handler = MenuHandler()
        self.menu_handler.insert("Delete", callback=self.delete_callback)
        self.server = server
        self.frame_id = "world"
        self.id = 0
        self.key = name + str(self.id)
        self.root_object = ""
        self.parent_map = {}
        self.marker_map = {}
        self.callback_map = {}
        self.end_effector_link_data = {}
        self.marker_pose_offset = {}
        self.display_objects = []
        self.waypoints = []
        self.structure = None

        # menu stuff
        self.marker_menus = {}
        self.menu_handles = {}

        # helper frames
        self.robotTroot = kdl.Frame()
        self.rootTobj = {}
        self.objTwp = {}
        self.wpTee = {}

        # not all templates will have a dynamic reconfigure server
        self.dserver = None
        self.initial_pose = initial_pose

        if not isinstance(robot_config, RobotConfig) :
            rospy.loginfo("AffordanceTemplate::init() -- problem setting robot config")
        else :
            self.robot_config = robot_config
            self.frame_id = self.robot_config.frame_id
            self.robotTroot = getFrameFromPose(self.robot_config.root_offset)

        # set up menu info
        self.waypoint_menu_options = []
        self.waypoint_menu_options.append(("Compute Backwards Path", True))
        self.waypoint_menu_options.append(("Execute On Move", True))
        self.waypoint_menu_options.append(("Execute", False))
        self.waypoint_menu_options.append(("Sync To Actual", False))
        self.waypoint_menu_options.append(("Stored Poses", False))

        self.object_menu_options = []
        self.object_menu_options.append(("Compute Backwards Path", True))
        self.object_menu_options.append(("Execute On Move", True))
        self.object_menu_options.append(("Execute", False))
        self.object_menu_options.append(("Reset", False))
        self.object_menu_options.append(("Save as..", False))


    def set_root_object(self, name) :
        self.root_object = name

    def get_root_object(self) :
        return self.root_object

    def add_interactive_marker(self, marker, callback=None):
        name = marker.name
        rospy.loginfo(str("Adding affordance template marker: " + name))
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

    def is_parent(self, child, parent) :
        if not child in self.parent_map :
            return False
        elif self.parent_map[child] == None :
            return False
        elif self.parent_map[child] == parent:
            return True
        else :
            return self.is_parent(self.parent_map[child], parent)

    def get_chain(self, parent, child) :
        if not self.is_parent(child, parent) :
            return kdl.Frame()
        else :
            if parent == self.parent_map[child] :
                T = self.rootTobj[child]
            else :
                T = self.get_chain(parent,self.parent_map[child])*self.rootTobj[child]
            return T

    def get_chain_from_robot(self, child) :
        return self.get_chain("robot",child)


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
            rospy.logerr("AffordanceTemplate::pose_from_origin() error")
        return p

    def create_from_structure(self) :

        if self.structure == None :
            return False

        self.key = self.structure.name + str(self.id)

        # parse objects
        ids = 0
        for obj in self.structure.display_objects.display_objects :

            if ids == 0:
                self.set_root_object(obj.name)
                self.parent_map[obj.name] = "robot"

            robotTfirst_object = self.robotTroot
            robotTroot = robotTfirst_object # this might be the case if root and first_object are the same

            if not obj.parent == None :
                self.parent_map[obj.name] = obj.parent
                robotTroot = robotTroot*self.get_chain_from_robot(obj.parent)

            self.rootTobj[obj.name] = getFrameFromPose(self.pose_from_origin(obj.origin))
            p = getPoseFromFrame(robotTroot*self.rootTobj[obj.name])

            int_marker = InteractiveMarker()
            control = InteractiveMarkerControl()

            self.marker_menus[obj.name] = MenuHandler()
            self.setup_object_menu(obj.name)
            # self.marker_menus[obj.name].setCheckState( self.menu_handles[(group,"Execute On Move")], MenuHandler.CHECKED )

            p0 = geometry_msgs.msg.Pose()
            p0.orientation.w = 1

            int_marker.header.frame_id = self.frame_id
            int_marker.pose = p
            int_marker.name = obj.name
            int_marker.description = obj.name
            int_marker.scale = obj.controls.scale

            control = InteractiveMarkerControl()
            marker = Marker()
            marker.ns = obj.name
            marker.id = ids

            if isinstance(obj.geometry, template_markers.atdf_parser.Mesh) :
                marker.type = Marker.MESH_RESOURCE
                marker.mesh_resource = obj.geometry.filename
                marker.mesh_use_embedded_materials = True
                marker.scale.x = obj.geometry.scale[0]
                marker.scale.y = obj.geometry.scale[1]
                marker.scale.z = obj.geometry.scale[2]
                # control = CreatePrimitiveControl(obj.name, p, 1.0, Marker.MESH_RESOURCE, ids)
            elif isinstance(obj.geometry, template_markers.atdf_parser.Box) :
                marker.type = Marker.CUBE
                marker.scale.x = obj.geometry.size[0]
                marker.scale.y = obj.geometry.size[1]
                marker.scale.z = obj.geometry.size[2]
            elif isinstance(obj.geometry, template_markers.atdf_parser.Sphere) :
                marker.type = Marker.SPHERE
                marker.scale.x = obj.geometry.radius
                marker.scale.y = obj.geometry.radius
                marker.scale.z = obj.geometry.radius
            elif isinstance(obj.geometry, template_markers.atdf_parser.Cylinder) :
                marker.type = Marker.CYLINDER
                marker.scale.x = obj.geometry.radius
                marker.scale.y = obj.geometry.radius
                marker.scale.z = obj.geometry.length

            control.markers.append(marker)

            if not obj.material == None :
                control.markers[0].color.r = obj.material.color.rgba[0]
                control.markers[0].color.g = obj.material.color.rgba[1]
                control.markers[0].color.b = obj.material.color.rgba[2]
                control.markers[0].color.a = obj.material.color.rgba[3]
                if isinstance(obj.geometry, template_markers.atdf_parser.Mesh) :
                    control.markers[0].mesh_use_embedded_materials = False

            scale = 1.0
            if obj.controls.scale != None :
                scale = obj.controls.scale

            # int_marker = CreateInteractiveMarker(self.frame_id, obj.name, scale)
            int_marker.controls.append(control)
            int_marker.controls.extend(CreateCustomDOFControls("",
                obj.controls.xyz[0], obj.controls.xyz[1], obj.controls.xyz[2],
                obj.controls.rpy[0], obj.controls.rpy[1], obj.controls.rpy[2]))

            self.add_interactive_marker(int_marker)
            self.marker_menus[obj.name].apply( self.server, obj.name )
            self.server.applyChanges()

            self.marker_map[obj.name] = control.markers[0]
            self.marker_pose_offset[obj.name] = self.pose_from_origin(obj.origin)

            self.display_objects.append(obj.name)

            ids += 1

        # parse end effector information
        wp_ids = 0
        for wp in self.structure.end_effector_waypoints.end_effector_waypoints :

            wp_name = str(wp.end_effector) + "." + str(wp.id)
            ee_name = self.robot_config.end_effector_name_map[int(wp.end_effector)]

            robotTroot = self.robotTroot*self.get_chain_from_robot(wp.display_object)

            if not wp.display_object in self.display_objects :
                rospy.logerr(str("AffordanceTemplate::create_from_structure() -- end-effector display object " + wp.display_object + "not found!"))

            wp_pose = self.pose_from_origin(wp.origin)
            ee_offset = self.robot_config.end_effector_pose_map[ee_name]

            self.objTwp[wp_name] = getFrameFromPose(wp_pose)
            self.wpTee[wp_name] = getFrameFromPose(ee_offset)

            display_pose = getPoseFromFrame(robotTroot*self.objTwp[wp_name])

            int_marker = InteractiveMarker()
            control = InteractiveMarkerControl()

            int_marker.header.frame_id = self.frame_id
            int_marker.pose = display_pose
            int_marker.name = wp_name
            int_marker.description = wp_name
            int_marker.scale = wp.controls.scale

            menu_control = InteractiveMarkerControl()
            menu_control.interaction_mode = InteractiveMarkerControl.BUTTON

            self.marker_menus[wp_name] = MenuHandler()
            self.setup_waypoint_menu(wp_name)
            # self.marker_menus[wp_name].setCheckState( self.menu_handles[(group,"Execute On Move")], MenuHandler.CHECKED )

            for m in self.robot_config.end_effector_markers[ee_name].markers :
                ee_m = copy.deepcopy(m)
                ee_m.header.frame_id = self.frame_id
                ee_m.pose = getPoseFromFrame(getFrameFromPose(display_pose)*self.wpTee[wp_name]*getFrameFromPose(m.pose))
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

            self.add_interactive_marker(int_marker)
            self.marker_menus[wp_name].apply( self.server, wp_name )
            self.server.applyChanges()

            # self.marker_pose_offset[wp_name] = display_pose
            self.waypoints.append(wp_name)

            if not wp.display_object == None :
                 self.parent_map[wp_name] = wp.display_object
            else :
                self.parent_map[wp_name] = self.get_root_object()
            wp_ids += 1

    def setup_object_menu(self, obj) :
        for m,c in self.object_menu_options :
            # if m == "Stored Poses" :
            #     sub_menu_handle = self.marker_menus[obj].insert(m)
            #     for p in self.moveit_interface.get_stored_state_list(obj) :
            #         self.menu_handles[(obj,m,p)] = self.marker_menus[obj].insert(p,parent=sub_menu_handle,callback=self.stored_pose_callback)
            # else :
            self.menu_handles[(obj,m)] = self.marker_menus[obj].insert( m, callback=self.process_feedback )
            if c : self.marker_menus[obj].setCheckState( self.menu_handles[(obj,m)], MenuHandler.UNCHECKED )

    def setup_waypoint_menu(self, waypoint) :
        for m,c in self.waypoint_menu_options :
            # if m == "Stored Poses" :
            #     sub_menu_handle = self.marker_menus[waypoint].insert(m)
            #     for p in self.moveit_interface.get_stored_state_list(waypoint) :
            #         self.menu_handles[(waypoint,m,p)] = self.marker_menus[waypoint].insert(p,parent=sub_menu_handle,callback=self.stored_pose_callback)
            # else :
            self.menu_handles[(waypoint,m)] = self.marker_menus[waypoint].insert( m, callback=self.process_feedback )
            if c : self.marker_menus[waypoint].setCheckState( self.menu_handles[(waypoint,m)], MenuHandler.UNCHECKED )

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

    def process_feedback(self, feedback):
        # print "\n--------------------------------"
        # print "Process Feedback on marker: ", feedback.marker_name

        replan_path = False
        for m in self.marker_map.keys() :
            if self.is_parent(m, feedback.marker_name) :
                if m in self.display_objects :

                    #print "------------\n(object) feeback marker: ", feedback.marker_name
                    #print "------------\nchild: ", m

                    robotTroot_new = getFrameFromPose(feedback.pose)
                    #print "------------\nrobotTroot_new: "
                    #print robotTroot_new

                    Tint = self.get_chain(feedback.marker_name,m)
                    #print "------------\nTint: (", feedback.marker_name, ",", m, ")"
                    #print Tint

                    T = robotTroot_new*Tint
                    #print "------------\nT: "
                    #print T

                    p = getPoseFromFrame(T)
                    self.server.setPose(m, p)

                if m in self.waypoints :

                    #print "------------\n(waypoint) feedback marker: ", feedback.marker_name
                    #print "------------\nchild: ", m

                    robotTobj_new = getFrameFromPose(feedback.pose)
                    #print "------------\nrobotTroot_new: "
                    #print robotTobj_new

                    Tint = self.get_chain(feedback.marker_name,self.parent_map[m])*self.objTwp[m]#*self.wpTee[m]
                    #print "------------\nTint: (", feedback.marker_name, ",", m, ")"
                    #print Tint

                    T = robotTobj_new*Tint
                    #print "------------\nT: "
                    #print T

                    p = getPoseFromFrame(T)
                    self.server.setPose(m, p)

        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP :

            # print "----------\n"
            # print "----------"
            # print "MOUSE UP [", feedback.marker_name , "]!!!!"

            TInverse = kdl.Frame()
            Tdelta = kdl.Frame()

            if feedback.marker_name in self.display_objects :

                # print "------------\nrootTobj (orig): "
                # print self.rootTobj[feedback.marker_name]

                robotTobj_new = getFrameFromPose(feedback.pose)
                robotTroot = self.robotTroot

                robotTroot = self.get_chain_from_robot(feedback.marker_name)
                T = self.robotTroot*robotTroot
                Tdelta = T.Inverse()*robotTobj_new
                self.rootTobj[feedback.marker_name] = self.rootTobj[feedback.marker_name]*Tdelta


            if feedback.marker_name in self.waypoints :

                rootTobj = self.rootTobj[self.parent_map[feedback.marker_name]]

                # print "------------\nobjTwp (orig): "
                # print self.objTwp[feedback.marker_name]

                # print "------------\nrootTobj: "
                # print rootTobj

                # print "------------\nwpTee: "
                # print self.wpTee[feedback.marker_name]

                rootTwp = rootTobj*self.objTwp[feedback.marker_name]
                # print "------------\n: "
                # print rootTwp

                robotTwp_new = getFrameFromPose(feedback.pose)
                robotTroot = self.get_chain_from_robot(self.parent_map[feedback.marker_name])*self.objTwp[feedback.marker_name]#*self.wpTee[feedback.marker_name]
                T = self.robotTroot*rootTobj*self.objTwp[feedback.marker_name]
                TInverse = T.Inverse()
                Tdelta = T.Inverse()*robotTwp_new
                self.objTwp[feedback.marker_name] = self.objTwp[feedback.marker_name]*Tdelta

                replan_path = True



            # print "------------\nself.robotTroot: "
            # print self.robotTroot

            # print "------------\nrobotTroot: "
            # print robotTroot

            # print "------------\nT: "
            # print T

            # print "------------\nTInverse: "
            # print TInverse

            # print "------------\nTdelta: "
            # print Tdelta


            # if feedback.marker_name in self.display_objects :
            #     print "------------\nrobotTobj_new: "
            #     print robotTobj_new

            #     print "------------\nrootTobj (updated): "
            #     print self.rootTobj[feedback.marker_name]

            # if feedback.marker_name in self.waypoints :
            #     print "------------\nrobotTee_new: "
            #     print robotTwp_new

            #     print "------------\nobjTwp (updated): "
            #     print self.objTwp[feedback.marker_name]

        self.server.applyChanges()


        if replan_path :

            print "Replanning path"
            print self.objTwp.keys()

            ee_name = self.robot_config.get_end_effector_name(int(feedback.marker_name.split(".")[0]))
            ee_id = self.robot_config.get_end_effector_name(int(feedback.marker_name.split(".")[1]))
            manipulator_name = self.robot_config.get_manipulator(ee_name)
            ee_offset = self.robot_config.end_effector_pose_map[ee_name]

            print "ee_name: ", ee_name
            print "ee_id: ", ee_id
            print "manipulator_name: ", manipulator_name
            self.robot_config.moveit_interface.groups[manipulator_name].clear_pose_targets()
            pose_targets = []

            for k in self.objTwp.keys() :
                if self.robot_config.get_end_effector_name(int(k.split(".")[0])) == ee_name :
                    # need to make sure things are in the right order, but dont worry about that now
                    pt = geometry_msgs.msg.PoseStamped()
                    pt.header = self.server.get(k).header
                    pt.pose = self.server.get(k).pose

                    T_goal = getFrameFromPose(pt.pose)
                    T_offset = getFrameFromPose(ee_offset)
                    T = T_goal*T_offset
                    pt.pose = getPoseFromFrame(T)

                    pose_targets.append(pt.pose)
                    print "trying to get moveit path!"
                    # self.robot_config.moveit_interface.create_plan_to_target(manipulator_name, pt)

            self.robot_config.moveit_interface.create_path_plan(manipulator_name, feedback.header.frame_id, pose_targets)


