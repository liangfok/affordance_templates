#!/usr/bin/env python
import argparse

import rospy
import roslib; roslib.load_manifest("template_markers")

from template_markers.atdf_parser import *

if __name__ == '__main__':

    rospy.init_node('template_parse_test')

    try:
		at = AffordanceTemplate.from_file("/home/shart/hydro_workspace/src/affordance_templates/affordance_template_library/templates/test.atdf")

		print "---------------------"
		print "---------------------"
		print "found new template:"
		print "  name:  ", at.name
		print "  image: ", at.image
		for obj in at.display_objects.display_objects :
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
		for wp in at.end_effector_waypoints.end_effector_waypoints :
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

    except rospy.ROSInterruptException:
        pass




