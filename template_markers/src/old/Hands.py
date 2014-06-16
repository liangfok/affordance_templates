#!/usr/bin/env python
import rospy
import sys
from TaskTemplate import TaskTemplate
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *

LEFT = "left"
RIGHT = "right"

class Hands(TaskTemplate):
    def __init__(self, server, id_):
        TaskTemplate.__init__(self, server, id_, "Hands", "/world")
        self.initROS()
        self.initMenu()
        self.initMarker()
        self.createHands()

    def initMarker(self):
        self._hands = self.createMarker(self.markerCB)
        self.attachMenuHandler(self._hands)
        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

    def markerCB(self, feedback):
        wTc_old = getFrameFromPose(self._hands.pose)
        wTc_new = getFrameFromPose(feedback.pose)
        # first get positions of all markers in case they've moved
        index = 0
        for hand in self._hands[LEFT]:
            wTh = getFrameFromPose(hand.getCurrentPose())
            cTh = wTc_old.Inverse()*wTh
            self._handPoseArray[LEFT][self._leftPathIndex].poses[index] = getPoseFromFrame(cTh)
            hand.setHandPose(getPoseFromFrame(wTc_new*cTh))
            index += 1

        index = 0
        for hand in self._hands[RIGHT]:
            wTh = getFrameFromPose(hand.getCurrentPose())
            cTh = wTc_old.Inverse()*wTh
            self._handPoseArray[RIGHT][self._leftPathIndex].poses[index] = getPoseFromFrame(cTh)
            hand.setHandPose(getPoseFromFrame(wTc_new*cTh))
            index += 1

        self._hands.pose = feedback.pose
        self.updateMarkerFrame(wTc_new)

    def createHands(self):
        self.addNewHand(LEFT)
        self.addNewHand(RIGHT)

    def hideControls(self):
        marker = self.getMarker(self._name + str(self._id))
        marker.controls[:] = [control for control in marker.controls if (control.name == 'TranslateZ')]

    def showControls(self):
        marker = self.getMarker(self._name + str(self._id))
        marker.controls = []
        marker.controls.extend(Create6DOFControls())
        self.attachMenuHandler(marker)
        self.server.insert(marker)
        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

if __name__ == '__main__':
    count = '0'
    # backwards compatibility, pass in True for second argument to force
    # each template node to publish to a different topic
    separate_topic = False
    if len(sys.argv) > 1:
        count = sys.argv[1]
        separate_topic = False
    if len(sys.argv) > 2:
        count = sys.argv[1]
        import ast
        separate_topic = ast.literal_eval(sys.argv[2])
    nodeKey = "Hands" + count
    rospy.init_node(nodeKey)
    if separate_topic:
        server = InteractiveMarkerServer(nodeKey)
    else:
        server = InteractiveMarkerServer("AffordanceTemplates", nodeKey)
    templateLeft = Hands(server, int(count))
    while not rospy.is_shutdown():
        rospy.spin()

