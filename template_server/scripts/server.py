#!/usr/bin/env python
import os
import sys
import rospy
from template_server import AffordanceTemplateServer

if __name__ == '__main__':
    # TODO: parge arguments properly
    topic_arg = None
    if len(sys.argv) > 1:
        topic_arg = str(sys.argv[1])

    rospy.init_node('AffordanceTemplateServer')

    server = AffordanceTemplateServer(topic_arg)
    server.start()
