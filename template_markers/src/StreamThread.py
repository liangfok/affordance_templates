#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from nav_msgs.msg import Path
from template_utilities import *
import PyKDL as kdl
import threading
import time

class StreamThread(threading.Thread):
    def __init__(self, markerFrame, frameIDIn, frameIDOut):
        threading.Thread.__init__(self)
        self._stopStreaming = False
        self._wTc = markerFrame
        self._leftStreamPosesPub = rospy.Publisher("/left_hand_stream/pose_stamped", PoseStamped)
        self._rightStreamPosesPub = rospy.Publisher("/right_hand_stream/pose_stamped", PoseStamped)
        self._leftPose = PoseStamped()
        self._rightPose = PoseStamped()
        self._frameID = frameIDIn
        self._frameOut = frameIDOut
        self._tfListener = tf.TransformListener()
        self._index = 0

    def setStopStreaming(self, flag):
        self._stopStreaming = flag

    def setMarkerFrame(self, frame):
        self._wTc = frame

    def publish(self):
        try:
            t = rospy.Time(0)
            self._tfListener.waitForTransform("/v1/LeftPalm", self._frameID, t, rospy.Duration(2))
            (leftPos, leftRot) = self._tfListener.lookupTransform(self._frameID, "/v1/LeftPalm", t)
            poseLeft = Pose()
            poseLeft.position.x = leftPos[0]
            poseLeft.position.y = leftPos[1]
            poseLeft.position.z = leftPos[2]
            poseLeft.orientation.x = leftRot[0]
            poseLeft.orientation.y = leftRot[1]
            poseLeft.orientation.z = leftRot[2]
            poseLeft.orientation.w = leftRot[3]
            poseStampedLeft = PoseStamped()
            poseStampedLeft.header.seq = self._index
            poseStampedLeft.header.stamp = rospy.Time.now()
            poseStampedLeft.header.frame_id = self._frameOut
            poseStampedLeft.pose = getPoseFromFrame(self._wTc.Inverse()*getFrameFromPose(poseLeft))
            # set in thread
            self._leftStreamPosesPub.publish(poseStampedLeft)
        except:
            print "did not get transform from /v1/LeftPalm to " + self._frameID
            pass
        try:
            t = rospy.Time(0)
            self._tfListener.waitForTransform("/v1/RightPalm", self._frameID, t, rospy.Duration(2))
            (rightPos, rightRot) = self._tfListener.lookupTransform(self._frameID, "/v1/RightPalm", t)
            poseRight = Pose()
            poseRight.position.x = rightPos[0]
            poseRight.position.y = rightPos[1]
            poseRight.position.z = rightPos[2]
            poseRight.orientation.x = rightRot[0]
            poseRight.orientation.y = rightRot[1]
            poseRight.orientation.z = rightRot[2]
            poseRight.orientation.w = rightRot[3]
            poseStampedRight = PoseStamped()
            poseStampedRight.header.seq = self._index
            poseStampedRight.header.stamp = rospy.Time.now()
            poseStampedRight.header.frame_id = self._frameOut
            poseStampedRight.pose = getPoseFromFrame(self._wTc.Inverse()*getFrameFromPose(poseRight))
            self._rightStreamPosesPub.publish(poseStampedRight)
        except:
            pass

        self._index += 1

    def run(self):

        while not self._stopStreaming:
            # try:
            #     t = rospy.Time(0)
            #     self._tfListener.waitForTransform("/v1/LeftPalm", self._frameID, t, rospy.Duration(2))
            #     (leftPos, leftRot) = self._tfListener.lookupTransform(self._frameID, "/v1/LeftPalm", t)
            #     poseLeft = Pose()
            #     poseLeft.position.x = leftPos[0]
            #     poseLeft.position.y = leftPos[1]
            #     poseLeft.position.z = leftPos[2]
            #     poseLeft.orientation.x = leftRot[0]
            #     poseLeft.orientation.y = leftRot[1]
            #     poseLeft.orientation.z = leftRot[2]
            #     poseLeft.orientation.w = leftRot[3]
            #     poseStampedLeft = PoseStamped()
            #     poseStampedLeft.header.seq = index
            #     poseStampedLeft.header.stamp = rospy.Time.now()
            #     poseStampedLeft.header.frame_id = self._frameOut
            #     poseStampedLeft.pose = getPoseFromFrame(self._wTc.Inverse()*getFrameFromPose(poseLeft))
            #     # set in thread
            #     self._leftStreamPosesPub.publish(poseStampedLeft)
            # except:
            #     pass
            # try:
            #     t = rospy.Time(0)
            #     self._tfListener.waitForTransform("/v1/RightPalm", self._frameID, t, rospy.Duration(2))
            #     (rightPos, rightRot) = self._tfListener.lookupTransform(self._frameID, "/v1/RightPalm", t)
            #     poseRight = Pose()
            #     poseRight.position.x = rightPos[0]
            #     poseRight.position.y = rightPos[1]
            #     poseRight.position.z = rightPos[2]
            #     poseRight.orientation.x = rightRot[0]
            #     poseRight.orientation.y = rightRot[1]
            #     poseRight.orientation.z = rightRot[2]
            #     poseRight.orientation.w = rightRot[3]
            #     poseStampedRight = PoseStamped()
            #     poseStampedRight.header.seq = index
            #     poseStampedRight.header.stamp = rospy.Time.now()
            #     poseStampedRight.header.frame_id = self._frameOut
            #     poseStampedRight.pose = getPoseFromFrame(self._wTc.Inverse()*getFrameFromPose(poseRight))
            #     self._rightStreamPosesPub.publish(poseStampedRight)
            # except:
            #     pass
            self.publish()
            time.sleep(0.1)