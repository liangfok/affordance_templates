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
        # self.end_effector_link_data = {}
        self.marker_pose_offset = {}
        self.display_objects = []

        self.waypoints = []
        self.structure = None
        self.robot_config = None

        # parameter storage
        self.object_origin = {}
        self.object_controls = {}
        self.object_geometry = {}
        self.object_material = {}

        self.waypoint_origin = {}
        self.waypoint_controls = {}
        self.waypoint_end_effectors = {}
        self.waypoint_ids = {}

        # menu stuff
        self.marker_menus = {}
        self.menu_handles = {}

        # control helper stuff
        self.waypoint_index = {}
        self.waypoint_backwards_flag = {}
        self.waypoint_auto_execute = {}
        self.waypoint_plan_valid = {}
        self.waypoint_loop = {}
        self.waypoint_max = {}
        self.waypoint_controls_display_on = True
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

        # set up menu info
        self.waypoint_menu_options = []
        self.waypoint_menu_options.append(("Display Next Path Segment", False))
        self.waypoint_menu_options.append(("Display Full Path", False))
        self.waypoint_menu_options.append(("Compute Backwards Path", True))
        self.waypoint_menu_options.append(("Execute On Move", True))
        self.waypoint_menu_options.append(("Execute Next Segment", False))
        self.waypoint_menu_options.append(("Execute Full Path", False))
        self.waypoint_menu_options.append(("Loop Path", True))
        self.waypoint_menu_options.append(("Sync To Actual", False))
        self.waypoint_menu_options.append(("Stored Poses", False))
        self.waypoint_menu_options.append(("Hide Controls", True))

        self.object_menu_options = []
        self.object_menu_options.append(("Display Next Path Segment", False))
        self.object_menu_options.append(("Display Full Path", False))
        self.object_menu_options.append(("Compute Backwards Path", True))
        self.object_menu_options.append(("Execute On Move", True))
        # self.object_menu_options.append(("Loop Path", True))
        self.object_menu_options.append(("Execute Next Segment", False))
        self.object_menu_options.append(("Execute Full Path", False))
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

    def remove_all_markers(self) :
        markers = copy.deepcopy(self.marker_map)
        for m in markers :
            self.remove_interactive_marker(m)

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

    def load_initial_parameters(self) :

        if self.robot_config == None :
            return False

        if self.structure == None :
            return False

        self.display_objects = []
        self.waypoints = []

        self.frame_id = self.robot_config.frame_id
        self.robotTroot = getFrameFromPose(self.robot_config.root_offset)

        self.key = self.structure.name + str(self.id)

        # parse objects
        ids = 0
        for obj in self.structure.display_objects.display_objects :

            self.display_objects.append(obj.name)

            if ids == 0:
                self.set_root_object(obj.name)
                self.parent_map[obj.name] = "robot"

            if not obj.parent == None :
                self.parent_map[obj.name] = obj.parent

            self.rootTobj[obj.name] = getFrameFromPose(self.pose_from_origin(obj.origin))
            self.marker_pose_offset[obj.name] = self.pose_from_origin(obj.origin)

            self.object_origin[obj.name] = obj.origin
            self.object_controls[obj.name] = obj.controls
            self.object_geometry[obj.name] = obj.geometry
            self.object_material[obj.name] = obj.material

            ids += 1

        # parse end effector information
        wp_ids = 0
        for wp in self.structure.end_effector_waypoints.end_effector_waypoints :

            wp_name = str(wp.end_effector) + "." + str(wp.id)
            ee_name = self.robot_config.end_effector_name_map[int(wp.end_effector)]

            self.waypoints.append(wp_name)

            if not wp.display_object in self.display_objects :
                rospy.logerr(str("AffordanceTemplate::create_from_structure() -- end-effector display object " + wp.display_object + "not found!"))

            wp_pose = self.pose_from_origin(wp.origin)
            ee_offset = self.robot_config.end_effector_pose_map[ee_name]

            self.objTwp[wp_name] = getFrameFromPose(wp_pose)
            self.wpTee[wp_name] = getFrameFromPose(ee_offset)

            self.waypoint_ids[wp_name] = wp.id
            self.waypoint_end_effectors[wp_name] = wp.end_effector
            self.waypoint_origin[wp_name] = wp.origin
            self.waypoint_controls[wp_name] = wp.controls

            if not wp.display_object == None :
                 self.parent_map[wp_name] = wp.display_object
            else :
                self.parent_map[wp_name] = self.get_root_object()

            if self.waypoint_end_effectors[wp_name] not in self.waypoint_index :
                id = int(self.waypoint_end_effectors[wp_name])
                self.waypoint_index[id] = -1
                self.waypoint_backwards_flag[id] = False
                self.waypoint_auto_execute[id] = False
                self.waypoint_plan_valid[id] = False
                self.waypoint_loop[id] = False
                self.waypoint_max[id] = int(self.waypoint_ids[wp_name])
            else :
                # set max wp_name id for this ee
                if int(self.waypoint_ids[wp_name]) > self.waypoint_max[id] :
                    self.waypoint_max[id] = int(self.waypoint_ids[wp_name])


            wp_ids += 1


    def create_from_parameters(self) :

        self.key = self.structure.name + str(self.id)

        # parse objects
        ids = 0
        debug_id = 0
        for obj in self.display_objects :

            robotTfirst_object = self.robotTroot
            robotTroot = robotTfirst_object # this might be the case if root and first_object are the same

            if obj in self.parent_map :
                robotTroot = robotTroot*self.get_chain_from_robot(self.parent_map[obj])

            self.rootTobj[obj] = getFrameFromPose(self.marker_pose_offset[obj])
            p = getPoseFromFrame(robotTroot*self.rootTobj[obj])

            int_marker = InteractiveMarker()
            control = InteractiveMarkerControl()

            self.marker_menus[obj] = MenuHandler()
            self.setup_object_menu(obj)

            p0 = geometry_msgs.msg.Pose()
            p0.orientation.w = 1

            int_marker.header.frame_id = self.frame_id
            int_marker.pose = p
            int_marker.name = obj
            int_marker.description = obj
            int_marker.scale = self.object_controls[obj].scale

            control = InteractiveMarkerControl()
            marker = Marker()
            marker.ns = obj
            marker.id = ids

            if isinstance(self.object_geometry[obj], template_markers.atdf_parser.Mesh) :
                marker.type = Marker.MESH_RESOURCE
                marker.mesh_resource = self.object_geometry[obj].filename
                marker.mesh_use_embedded_materials = True
                marker.scale.x = self.object_geometry[obj].scale[0]
                marker.scale.y = self.object_geometry[obj].scale[1]
                marker.scale.z = self.object_geometry[obj].scale[2]
                # control = CreatePrimitiveControl(obj.name, p, 1.0, Marker.MESH_RESOURCE, ids)
            elif isinstance(self.object_geometry[obj], template_markers.atdf_parser.Box) :
                marker.type = Marker.CUBE
                marker.scale.x = self.object_geometry[obj].size[0]
                marker.scale.y = self.object_geometry[obj].size[1]
                marker.scale.z = self.object_geometry[obj].size[2]
            elif isinstance(self.object_geometry[obj], template_markers.atdf_parser.Sphere) :
                marker.type = Marker.SPHERE
                marker.scale.x = self.object_geometry[obj].radius
                marker.scale.y = self.object_geometry[obj].radius
                marker.scale.z = self.object_geometry[obj].radius
            elif isinstance(self.object_geometry[obj], template_markers.atdf_parser.Cylinder) :
                marker.type = Marker.CYLINDER
                marker.scale.x = self.object_geometry[obj].radius
                marker.scale.y = self.object_geometry[obj].radius
                marker.scale.z = self.object_geometry[obj].length

            control.markers.append(marker)

            if obj in self.object_material :
                if self.object_material[obj] != None :
                    control.markers[0].color.r = self.object_material[obj].color.rgba[0]
                    control.markers[0].color.g = self.object_material[obj].color.rgba[1]
                    control.markers[0].color.b = self.object_material[obj].color.rgba[2]
                    control.markers[0].color.a = self.object_material[obj].color.rgba[3]
                    if isinstance(obj.geometry, template_markers.atdf_parser.Mesh) :
                        control.markers[0].mesh_use_embedded_materials = False

            scale = 1.0
            if obj in self.object_controls :
                scale = self.object_controls[obj]

            # int_marker = CreateInteractiveMarker(self.frame_id, obj.name, scale)
            int_marker.controls.append(control)
            int_marker.controls.extend(CreateCustomDOFControls("",
                self.object_controls[obj].xyz[0], self.object_controls[obj].xyz[1], self.object_controls[obj].xyz[2],
                self.object_controls[obj].rpy[0], self.object_controls[obj].rpy[1], self.object_controls[obj].rpy[2]))

            self.add_interactive_marker(int_marker)
            self.marker_menus[obj].apply( self.server, obj )
            self.server.applyChanges()

            self.marker_map[obj] = control.markers[0]
            self.marker_pose_offset[obj] = self.pose_from_origin(self.object_origin[obj])

            ids += 1


        # parse end effector information
        wp_ids = 0
        for wp in self.waypoints :

            ee_name = self.robot_config.end_effector_name_map[int(self.waypoint_end_effectors[wp])]

            robotTroot = self.robotTroot*self.get_chain_from_robot(self.parent_map[wp])

            if not self.parent_map[wp] in self.display_objects :
                rospy.logerr(str("AffordanceTemplate::create_from_structure() -- end-effector display object " + self.parent_map[wp] + "not found!"))

            display_pose = getPoseFromFrame(robotTroot*self.objTwp[wp])

            int_marker = InteractiveMarker()
            control = InteractiveMarkerControl()

            int_marker.header.frame_id = self.frame_id
            int_marker.pose = display_pose
            int_marker.name = wp
            int_marker.description = wp
            int_marker.scale = self.waypoint_controls[wp].scale

            menu_control = InteractiveMarkerControl()
            menu_control.interaction_mode = InteractiveMarkerControl.BUTTON

            self.marker_menus[wp] = MenuHandler()
            self.setup_waypoint_menu(wp)

            id = int(self.waypoint_end_effectors[wp])
            if self.waypoint_backwards_flag[id] :
                self.marker_menus[wp].setCheckState( self.menu_handles[(wp,"Compute Backwards Path")], MenuHandler.CHECKED )
            if self.waypoint_auto_execute[id] :
                self.marker_menus[wp].setCheckState( self.menu_handles[(wp,"Execute On Move")], MenuHandler.CHECKED )
            if self.waypoint_loop[id] :
                self.marker_menus[wp].setCheckState( self.menu_handles[(wp,"Loop Path")], MenuHandler.CHECKED )
            if self.waypoint_controls_display_on :
                self.marker_menus[wp].setCheckState( self.menu_handles[(wp,"Hide Controls")], MenuHandler.UNCHECKED )
            else :
                self.marker_menus[wp].setCheckState( self.menu_handles[(wp,"Hide Controls")], MenuHandler.CHECKED )

            for m in self.robot_config.end_effector_markers[ee_name].markers :
                ee_m = copy.deepcopy(m)
                ee_m.header.frame_id = self.frame_id
                ee_m.pose = getPoseFromFrame(getFrameFromPose(display_pose)*self.wpTee[wp]*getFrameFromPose(m.pose))
                ee_m.color.r = .2
                ee_m.color.g = .5
                ee_m.color.b = .2
                ee_m.color.a = .8
                menu_control.markers.append( ee_m )


            scale = 1.0
            if wp in self.waypoint_controls :
                scale = self.waypoint_controls[wp].scale

            int_marker.controls.append(menu_control)

            if(self.waypoint_controls_display_on) :
                int_marker.controls.extend(CreateCustomDOFControls("",
                    self.waypoint_controls[wp].xyz[0], self.waypoint_controls[wp].xyz[1], self.waypoint_controls[wp].xyz[2],
                    self.waypoint_controls[wp].rpy[0], self.waypoint_controls[wp].rpy[1], self.waypoint_controls[wp].rpy[2]))

            self.add_interactive_marker(int_marker)
            self.marker_menus[wp].apply( self.server, wp )
            self.server.applyChanges()

            # self.marker_pose_offset[wp] = display_pose

            wp_ids += 1


    def create_from_structure(self) :
        self.load_initial_parameters()
        self.create_from_parameters()


    def setup_object_menu(self, obj) :
        for m,c in self.object_menu_options :
            self.menu_handles[(obj,m)] = self.marker_menus[obj].insert( m, callback=self.process_feedback )
            if c : self.marker_menus[obj].setCheckState( self.menu_handles[(obj,m)], MenuHandler.UNCHECKED )

    def setup_waypoint_menu(self, waypoint) :
        for m,c in self.waypoint_menu_options :
            self.menu_handles[(waypoint,m)] = self.marker_menus[waypoint].insert( m, callback=self.process_feedback )
            if c : self.marker_menus[waypoint].setCheckState( self.menu_handles[(waypoint,m)], MenuHandler.UNCHECKED )

    def load_from_file(self, filename) :
        self.structure = template_markers.atdf_parser.AffordanceTemplateStructure.from_file(filename)
        # self.create_from_structure()

        self.load_initial_parameters()
        self.create_from_parameters()

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
                Tstore = robotTobj_new
                robotTroot = self.robotTroot

                robotTroot = self.get_chain_from_robot(feedback.marker_name)
                T = self.robotTroot*robotTroot
                Tdelta = T.Inverse()*robotTobj_new
                self.rootTobj[feedback.marker_name] = self.rootTobj[feedback.marker_name]*Tdelta

                if feedback.marker_name == self.get_root_object() :
                    self.robotTroot = getFrameFromPose(feedback.pose)*Tdelta*self.rootTobj[feedback.marker_name].Inverse()

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

                self.waypoint_origin[feedback.marker_name] = self.objTwp[feedback.marker_name]

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


        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:

            handle = feedback.menu_entry_id
            print "--------------"

            if feedback.marker_name in self.display_objects :

                print "object menu"
                ee_list = self.waypoint_index.keys()
                print "ee_list: ", ee_list

                if handle == self.menu_handles[(feedback.marker_name,"Reset")] :
                    rospy.loginfo(str("AffordanceTemplate::process_feedback() -- Reseting Affordance Template"))
                    self.create_from_structure()

            else :
                ee_list =[int(feedback.marker_name.split(".")[0])]
                waypoint_id = int(feedback.marker_name.split(".")[1])

            print ee_list
            for ee_id in ee_list :

                print "Waypoint Menu\n--------------------"

                ee_name = self.robot_config.get_end_effector_name(ee_id)
                manipulator_name = self.robot_config.get_manipulator(ee_name)
                ee_offset = self.robot_config.end_effector_pose_map[ee_name]
                max_idx = self.waypoint_max[ee_id]

                print "ee_name: ", ee_name
                print "ee_id: ", ee_id
                print "max wp ", max_idx
                # print "selected waypoint id: ", waypoint_id
                print "stored waypoint idx: ", self.waypoint_index[ee_id]
                print "manipulator_name: ", manipulator_name

                # compute plan idx stuff always for now
                if self.waypoint_index[ee_id] < 0 :
                    # haven't started yet, so set first waypoint to 0
                    next_path_idx = 0
                else :
                    if self.waypoint_backwards_flag[ee_id] :
                        next_path_idx = self.waypoint_index[ee_id]-1
                        if self.waypoint_loop[ee_id] :
                            if next_path_idx < 0 :
                                next_path_idx = max_idx
                        else :
                            if next_path_idx < 0 :
                                next_path_idx = 0
                    else :
                        next_path_idx = self.waypoint_index[ee_id]+1
                        if self.waypoint_loop[ee_id] :
                            if  next_path_idx > max_idx :
                                next_path_idx = 0
                        else :
                            if  next_path_idx > max_idx :
                                next_path_idx = max_idx

                print "next waypoint id: ", next_path_idx

                if handle == self.menu_handles[(feedback.marker_name,"Display Next Path Segment")] :
                    next_path_str = str(str(ee_id) + "." + str(next_path_idx))
                    if not next_path_str in self.objTwp :
                        rospy.logerr(str("AffordanceTemplate::process_feedback() -- path index[" + str(next_path_str) + "] not found!!"))
                    else :
                        rospy.loginfo(str("AffordanceTemplate::process_feedback() -- computing path to index[" + str(next_path_str) + "]"))
                        k = str(next_path_str)
                        pt = geometry_msgs.msg.PoseStamped()
                        pt.header = self.server.get(k).header
                        pt.pose = self.server.get(k).pose

                        T_goal = getFrameFromPose(pt.pose)
                        T_offset = getFrameFromPose(ee_offset)
                        T = T_goal*T_offset
                        pt.pose = getPoseFromFrame(T)

                        # self.robot_config.moveit_interface.groups[manipulator_name].clear_pose_targets()
                        self.robot_config.moveit_interface.create_plan_to_target(manipulator_name, pt)
                        self.waypoint_plan_valid[ee_id] = True

                if handle == self.menu_handles[(feedback.marker_name,"Display Full Path")] :
                    waypoints = []
                    frame_id = ""
                    print "waypoint_index: ", self.waypoint_index[ee_id]
                    print "next_path_idx: ", next_path_idx
                    print "max_idx: ", max_idx

                    r = range(0, max_idx+1)
                    if self.waypoint_backwards_flag[ee_id] :
                        r.reverse()

                    print "range: ", r
                    for idx in r :
                        next_path_str = str(str(ee_id) + "." + str(idx))
                        if not next_path_str in self.objTwp :
                            rospy.logerr(str("AffordanceTemplate::process_feedback() -- path index[" + str(next_path_str) + "] not found!!"))
                        else :
                            rospy.loginfo(str("AffordanceTemplate::process_feedback() -- computing path to index[" + str(next_path_str) + "]"))
                            k = str(next_path_str)
                            pt = geometry_msgs.msg.PoseStamped()
                            pt.header = self.server.get(k).header
                            pt.pose = self.server.get(k).pose
                            frame_id =  pt.header.frame_id

                            T_goal = getFrameFromPose(pt.pose)
                            T_offset = getFrameFromPose(ee_offset)
                            T = T_goal*T_offset
                            pt.pose = getPoseFromFrame(T)
                            waypoints.append(pt.pose)
                    self.robot_config.moveit_interface.create_path_plan(manipulator_name, frame_id, waypoints)
                    self.waypoint_plan_valid[ee_id] = True

                if handle == self.menu_handles[(feedback.marker_name,"Execute Next Segment")] :
                    if self.waypoint_plan_valid[ee_id] :
                        r = self.robot_config.moveit_interface.execute_plan(manipulator_name,from_stored=True)
                        if not r :
                            rospy.logerr(str("RobotTeleop::process_feedback(mouse) -- failed moveit execution for group: " + manipulator_name + ". re-synching..."))
                        self.waypoint_index[ee_id] = next_path_idx
                        rospy.loginfo(str("setting current waypoint idx: " + str(self.waypoint_index[ee_id])))
                        self.waypoint_plan_valid[ee_id] = False

                if handle == self.menu_handles[(feedback.marker_name,"Execute Full Path")] :
                    if self.waypoint_plan_valid[ee_id] :
                        r = self.robot_config.moveit_interface.execute_plan(manipulator_name,from_stored=True)
                        if not r :
                            rospy.logerr(str("RobotTeleop::process_feedback(mouse) -- failed moveit execution for group: " + manipulator_name + ". re-synching..."))
                        rospy.loginfo(str("setting current waypoint idx: " + str(self.waypoint_index[ee_id])))
                        self.waypoint_plan_valid[ee_id] = False

                if handle == self.menu_handles[(feedback.marker_name,"Compute Backwards Path")] :
                    state = self.marker_menus[feedback.marker_name].getCheckState( handle )
                    if state == MenuHandler.CHECKED:
                        self.marker_menus[feedback.marker_name].setCheckState( handle, MenuHandler.UNCHECKED )
                        self.waypoint_backwards_flag[ee_id] = False
                    else :
                        self.marker_menus[feedback.marker_name].setCheckState( handle, MenuHandler.CHECKED )
                        self.waypoint_backwards_flag[ee_id] = True
                    rospy.loginfo(str("AffordanceTemplate::process_feedback() -- setting Backwards Path flag for ee[" + str(ee_id) + "] to " + str(self.waypoint_backwards_flag[ee_id])))

                if handle == self.menu_handles[(feedback.marker_name,"Execute On Move")] :
                    state = self.marker_menus[feedback.marker_name].getCheckState( handle )
                    if state == MenuHandler.CHECKED:
                        self.marker_menus[feedback.marker_name].setCheckState( handle, MenuHandler.UNCHECKED )
                        self.waypoint_auto_execute[ee_id] = False
                    else :
                        self.marker_menus[feedback.marker_name].setCheckState( handle, MenuHandler.CHECKED )
                        self.waypoint_auto_execute[ee_id] = True
                    rospy.loginfo(str("AffordanceTemplate::process_feedback() -- setting AutoExecute flag for ee[" + str(ee_id) + "] to " + str(self.waypoint_auto_execute[ee_id])))

                # waypoint specific menu options
                if not feedback.marker_name in self.display_objects :

                    if handle == self.menu_handles[(feedback.marker_name,"Hide Controls")] :
                        state = self.marker_menus[feedback.marker_name].getCheckState( handle )
                        if state == MenuHandler.CHECKED:
                            self.marker_menus[feedback.marker_name].setCheckState( handle, MenuHandler.UNCHECKED )
                            self.waypoint_controls_display_on = True
                        else :
                            self.marker_menus[feedback.marker_name].setCheckState( handle, MenuHandler.CHECKED )
                            self.waypoint_controls_display_on = False

                        self.remove_all_markers()
                        self.create_from_parameters()

                        rospy.loginfo(str("AffordanceTemplate::process_feedback() -- setting Hide Controls flag to " + str(self.waypoint_controls_display_on)))

                    if handle == self.menu_handles[(feedback.marker_name,"Loop Path")] :
                        state = self.marker_menus[feedback.marker_name].getCheckState( handle )
                        if state == MenuHandler.CHECKED:
                            self.marker_menus[feedback.marker_name].setCheckState( handle, MenuHandler.UNCHECKED )
                            self.waypoint_loop[ee_id] = False
                        else :
                            self.marker_menus[feedback.marker_name].setCheckState( handle, MenuHandler.CHECKED )
                            self.waypoint_loop[ee_id] = True
                        rospy.loginfo(str("AffordanceTemplate::process_feedback() -- setting Loop flag for ee[" + str(ee_id) + "] to " + str(self.waypoint_loop[ee_id])))

            self.marker_menus[feedback.marker_name].reApply( self.server )

        self.server.applyChanges()



