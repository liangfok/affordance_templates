#!/usr/bin/env python
import roslib
roslib.load_manifest('marker_templates')
import rospy
import sys
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, PoseArray
from nav_msgs.msg import Path
from StringIO import StringIO

# Node example class.
class PathSerializer():

    # Must have __init__(self) function for a class, similar to a C++ class constructor.

    def __init__(self):
        # Get the ~private namespace parameters from command li
        self.poseArray = PoseArray()
        self.filename = "/home/shart/marker_trajectories/hose/hose_traj_1.txt"
        self.yaml = "/home/shart/marker_trajectories/hose/hose_traj_1.yaml"

    def setFile(self, fname) :
        self.filename = fname
        self.yaml = fname[0:fname.find('.txt')] + ".yaml"

    def appendPose(self, data) :
        print "got new pose in frame: ", data.header.frame_id
        self.poseArray.header.frame_id = data.header.frame_id
        self.poseArray.poses.append(data.pose)
        foo = StringIO()
        self.poseArray.serialize(foo)

        with open(self.yaml, 'a') as f:
            f.write(str(data))

        with open(self.filename, 'w') as f:
            f.write(foo.getvalue())


if __name__ == '__main__':
    rospy.init_node('path_serializer')

    f = rospy.get_param('~path_file', "/home/shart/marker_trajectories/hose/hose_traj_1.txt")
    pose_topic = rospy.get_param('~pose_topic', "/right_hand_pose")

    rospy.loginfo("PathSerializer: ")
    rospy.loginfo("\tpath_file: " + f)
    rospy.loginfo("\tpose_topic: " + pose_topic)

    try:
        ps = PathSerializer()
        ps.setFile(f)
        rospy.Subscriber(pose_topic, PoseStamped, ps.appendPose)

        while not rospy.is_shutdown():
            rospy.sleep(0.5)

    except rospy.ROSInterruptException: pass