#!/usr/bin/env python
import roslib
roslib.load_manifest('marker_templates')
import rospy
import sys
import time 
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, PoseArray
from nav_msgs.msg import Path
from StringIO import StringIO
from PyKDL import Frame

from marker_templates.util import *

# Node example class.
class PathPublisher():

    # Must have __init__(self) function for a class, similar to a C++ class constructor.

    def __init__(self):
        # Get the ~private namespace parameters from command li
        self.poseArray = PoseArray()
        self.waypointArray = PoseArray()
        self.filename = "/home/shart/marker_trajectories/hose/hose_waypoints_1.txt"
        self.pathPub = rospy.Publisher("/path_publisher/path", Path)
        self.posePub = rospy.Publisher("/path_publisher/poseArray", PoseArray)
        self.waypointPub = rospy.Publisher("/path_publisher/waypointArray", PoseArray)
        self.num_points = 10
        self.frameOffset = Frame()

    def readPoseArray(self) :
        with open(self.filename) as f:
            pa = f.read()
        self.waypointArray.deserialize(pa)
        [self.path, self.poseArray] = processPath(self.waypointArray, self.num_points)
        print "path size: ", len(self.poseArray.poses)

        self.waypointArray.header.frame_id = self.frame_id
        self.poseArray.header.frame_id = self.frame_id
        self.path.header.frame_id = self.frame_id

        self.waypointArray.header.stamp = rospy.Time.now()
        self.poseArray.header.frame_id = rospy.Time.now()
        self.path.header.stamp = rospy.Time.now()

    def publishPath(self) :

        self.waypointArray.header.stamp = rospy.Time.now()
        self.poseArray.header.frame_id = rospy.Time.now()
        self.path.header.stamp = rospy.Time.now()

        self.pathPub.publish(self.path)
        self.posePub.publish(self.poseArray)
        self.waypointPub.publish(self.waypointArray)

    def setFile(self, fname) :
        self.filename = fname

    def setFrameID(self, fid) :
        self.frame_id = fid

    def setNumPoints(self, N) :
        self.num_points = N

    def applyFrameOffset(self, pose) :
        self.frameOffset = getFrameFromPose(pose)

        for i in range(len(self.poseArray.poses)) :
            F = getFrameFromPose(self.poseArray.poses[i])
            Fnew = self.frameOffset*F
            self.poseArray.poses[i] = getPoseFromFrame(Fnew)
            self.path.poses[i].pose = self.poseArray.poses[i]

        for i in range(len(self.waypointArray.poses)) :
            F = getFrameFromPose(self.waypointArray.poses[i])
            Fnew = self.frameOffset*F
            self.waypointArray.poses[i] = getPoseFromFrame(Fnew)

        for i in range(len(self.path.poses)) :
            self.path.poses[i].header.frame_id = self.frame_id

if __name__ == '__main__':
    rospy.init_node('path_publisher')

    f = rospy.get_param('~path_file', "/home/shart/marker_trajectories/hose/hose_traj_1.path")
    frame_id = rospy.get_param('~frame_id', "/world")
    N = rospy.get_param('~num_points', 5)

    rospy.loginfo("PathPublisher: ")
    rospy.loginfo("\tpath_file: " + f)
    rospy.loginfo("\tframe_id: " + frame_id)
    rospy.loginfo("\tnum_points: " + str(N))

    try:
        pp = PathPublisher()
        pp.setFile(f)
        pp.setFrameID(frame_id)
        pp.setNumPoints(N)
        pp.readPoseArray()

        pose = Pose()
        pose.position.x = 0.4
        pose.position.y = -0.1
        pose.position.z = 1.87
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 1
        pose.orientation.w = 0
        pp.applyFrameOffset(pose)

        while not rospy.is_shutdown():
            pp.publishPath()
            time.sleep(0.5)
            #rospy.spin()

    except rospy.ROSInterruptException: pass