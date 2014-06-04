#!/usr/bin/env python
import argparse

import rospy
import roslib; roslib.load_manifest("template_markers")

from template_markers.atdf_parser import *

if __name__ == '__main__':

    rospy.init_node('template_parse_test')

    try:
		at = AffordanceTemplate.from_file("/home/shart/hydro_workspace/src/affordance_templates/affordance_template_library/templates/test.atdf")
    except rospy.ROSInterruptException:
        pass




