#!/usr/bin/env python
import argparse
import PyKDL as kdl

import rospy
import rospkg
import roslib; roslib.load_manifest("template_markers")

import geometry_msgs.msg
# import visualization_msgs.msg

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *


from template_markers.template_utilities import *
from template_markers.atdf_parser import *

class AffordanceTemplate(object) :

    def __init__(self, server, id, name, initial_pose=None):
        self.menu_handler = MenuHandler()
        self.menu_handler.insert("Delete", callback=self.deleteCallback)
        self.server = server
        self.key = name + str(id)
        self.marker_map = {}
        self.callback_map = {}
        # not all templates will have a dynamic reconfigure server
        self.dserver = None
        self.initial_pose = initial_pose

    # @property
    # def initial_pose(self):
    #     """Return a marker's initial pose if it was passed in."""
    #     return self.initial_pose

    # # @property
    # # def key(self):
    # #     """Get the marker template's key on the server."""
    # #     key = self.key + str(self.id)
    # #     return key

    # @property
    # def server(self):
    #     """Get the marker template's interactive marker server."""
    #     return self.server

    # @property
    # def marker_map(self):
    #     """Get the interactive marker map of this marker template."""
    #     return self.marker_map

    # @property
    # def callback_map(self):
    #     """Get the callback map of this marker template."""
    #     return self.callback_map

    # @property
    # def menu_handler(self):
    #     """Get the menu handler of this marker template."""
    #     return self.menu_handler

    def addInteractiveMarker(self, marker, callback=None):
        name = marker.name
        self.marker_map[name] = marker
        if callback:
            self.callback_map[name] = callback
            return self.server.insert(marker, callback)
        else:
            self.callback_map[name] = self.processFeedback
            return self.server.insert(marker, self.processFeedback)

    def removeInteractiveMarker(self, marker_name):
        if self.server.erase(marker_name):
            if marker_name in self.marker_map:
                del self.marker_map[marker_name]
                del self.callback_map[marker_name]
                return True
        return False

    def attachMenuHandler(self, marker):
        return self.menu_handler.apply(self.server, marker.name)

    def getMarker(self):
        return self.server.get(self._key)

    def hasMarker(self):
        if self._key in self.marker_map.keys():
            return True
        else:
            return False

    def deleteCallback(self, feedback):
        for key in self.marker_map.keys():
            self.server.erase(key)
        self.server.applyChanges()
        self.tearDown()

    def tearDown(self, keepAlive=False):
        # forcefully shut down service before killing node
        if self._dserver:
            self._dserver.set_service.shutdown("User deleted template.")
        # for rostest (and potentially other cases), we want to clean up but keep the node alive
        if not keepAlive:
            rospy.signal_shutdown("User deleted template.")

    def processFeedback(self, feedback):
        self.server.applyChanges()


class AffordanceTemplateCreator(object) :
    def __init__(self,server) :
        self.structure = None
        self.affordamce_template = None
        self.server = server

    def createFromStructure(self) :

        if self.structure == None :
            return False

        self.affordance_template = AffordanceTemplate(self.server, 0, self.structure.name)

        ids = 0
        frame_id = "world"
        for obj in self.structure.display_objects.display_objects :

            control = InteractiveMarkerControl()

            p = geometry_msgs.msg.Pose()
            q = (kdl.Rotation.RPY(obj.origin.rpy[0],obj.origin.rpy[1],obj.origin.rpy[2])).GetQuaternion()
            p.position.x = obj.origin.xyz[0]
            p.position.y = obj.origin.xyz[1]
            p.position.z = obj.origin.xyz[2]
            p.orientation.x = q[0]
            p.orientation.y = q[1]
            p.orientation.z = q[2]
            p.orientation.w = q[3]

            if isinstance(obj.geometry, Mesh) :
                control = CreatePrimitiveControl(obj.name, p, 1.0, Marker.MESH_RESOURCE, ids)
                control.markers[0].mesh_resource = obj.geometry.filename
                control.markers[0].mesh_use_embedded_materials = True
            elif isinstance(obj.geometry, Box) :
                control = CreatePrimitiveControl(obj.name, p, 1.0, Marker.CUBE, ids)
                control.markers[0].scale.x = obj.geometry.size[0]
                control.markers[0].scale.y = obj.geometry.size[1]
                control.markers[0].scale.z = obj.geometry.size[2]
            elif isinstance(obj.geometry, Sphere) :
                control = CreatePrimitiveControl(obj.name, p, 1.0, Marker.SPHERE, ids)
                control.markers[0].scale.x = obj.geometry.radius
                control.markers[0].scale.y = obj.geometry.radius
                control.markers[0].scale.z = obj.geometry.radius
            elif isinstance(obj.geometry, Cylinder) :
                control = CreatePrimitiveControl(obj.name, p, 1.0, Marker.CYLINDER, ids)
                control.markers[0].scale.x = obj.geometry.radius
                control.markers[0].scale.y = obj.geometry.radius
                control.markers[0].scale.z = obj.geometry.length

            control.markers[0].header.frame_id = frame_id

            if not obj.material == None :
                control.markers[0].color.r = obj.material.color.rgba[0]
                control.markers[0].color.g = obj.material.color.rgba[1]
                control.markers[0].color.b = obj.material.color.rgba[2]
                control.markers[0].color.a = obj.material.color.rgba[3]

            control.markers[0].action = Marker.ADD
            print control.markers[0]

            int_marker = CreateInteractiveMarker(frame_id, obj.name, .25)
            int_marker.controls.append(control)
            int_marker.controls.extend(Create6DOFControls())

            self.affordance_template.marker_map[obj.name] = control.markers[0]
            ids += 1

            self.affordance_template.addInteractiveMarker(int_marker)
            self.server.applyChanges()


    def loadFromFile(self, filename) :
        self.structure = AffordanceTemplateStructure.from_file(filename)
        self.createFromStructure()

        return self.structure

    def printStructure(self) :
        print "---------------------"
        print "---------------------"
        print "found new template:"
        print "  name:  ", self.structure.name
        print "  image: ", self.structure.image
        for obj in self.structure.display_objects.display_objects :
            print "---------------------"
            print "\tobject: ", obj.name
            print "----------------"
            print "\torigin xyz: ", obj.origin.xyz
            print "\torigin rpy: ", obj.origin.rpy
            print "----------------"
            print "\tcontrols xyz: ", obj.controls.xyz
            print "\tcontrols rpy: ", obj.controls.rpy
            print "----------------"
            if isinstance(obj.geometry, Box) :
                print "\tBox size: ", obj.geometry.size
            if isinstance(obj.geometry, Sphere) :
                print "\tSphere size: ", obj.geometry.size
            if isinstance(obj.geometry, Cylinder) :
                print "\tCylinder size: ", obj.geometry.size
            if isinstance(obj.geometry, Mesh) :
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
    path = rospack.get_path('affordance_template_library')

    server = InteractiveMarkerServer("affrodance_template_server")

    try:
        filename = path + "/templates/test.atdf"
        print filename

        ats = AffordanceTemplateCreator(server)
        at = ats.loadFromFile(filename)
        # ats.printStructure()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass




