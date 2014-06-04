#!/usr/bin/env python
import rospy
import roslib; roslib.load_manifest('affordance_templates')
from TaskTemplate import *
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *

class BallValve(TaskTemplate):
    def __init__(self, server, id_):
        TaskTemplate.__init__(self, server, id_, "/world", "BallValve")
        self.initROS()
        self.initMenu()
        self.initMarker()
        self.initHands()

    def initMarker(self):
        color = ColorRGBA(0.0, 0.5, 0.75, 0.5)
        offset = Pose()
        offset.position.x = 0
        offset.position.y = 0
        offset.position.z = -0.1
        rot = kdl.Rotation.RPY(0, 0, 0)
        q = rot.GetQuaternion()
        offset.orientation.x = q[0]
        offset.orientation.y = q[1]
        offset.orientation.z = q[2]
        offset.orientation.w = q[3]

        path = "package://marker_templates/resources/models/ball_valve.dae"

        scale = Vector3()
        scale.x = 1.0
        scale.y = 1.0
        scale.z = 1.0

        self.setMarkerColor(color)
        self.setMesh(path, offset, scale)

        self._ballValveMarker = self.createMarker(self.markerCB)
        self.attachMenuHandler(self._ballValveMarker)
        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

    def markerCB(self, feedback):
        wTc_old = getFrameFromPose(self._ballValveMarker.pose)
        wTc_new = getFrameFromPose(feedback.pose)
        # first get positions of all markers in case they've moved
        index = 0
        for hand in self._hands[LEFT]:
            frame = self._handPoseArray[LEFT][self._leftPathIndex].header.frame_id
            if frame == "BallValve" or frame == "":
                wTh = getFrameFromPose(hand.getCurrentPose())
                cTh = wTc_old.Inverse()*wTh
                self._handPoseArray[LEFT][self._leftPathIndex].poses[index] = getPoseFromFrame(cTh)
                hand.setHandPose(getPoseFromFrame(wTc_new*cTh))
            index += 1

        index = 0
        for hand in self._hands[RIGHT]:
            frame = self._handPoseArray[RIGHT][self._rightPathIndex].header.frame_id
            if frame == "BallValve" or frame == "":
                wTh = getFrameFromPose(hand.getCurrentPose())
                cTh = wTc_old.Inverse()*wTh
                self._handPoseArray[RIGHT][self._rightPathIndex].poses[index] = getPoseFromFrame(cTh)
                hand.setHandPose(getPoseFromFrame(wTc_new*cTh))
                index += 1

        self._ballValveMarker.pose = feedback.pose
        self.updateMarkerFrame(wTc_new)
   
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
    nodeKey = "BallValve" + count
    rospy.init_node(nodeKey)
    if separate_topic:
        server = InteractiveMarkerServer(nodeKey)
    else:
        server = InteractiveMarkerServer("MarkerTemplates", nodeKey)
    template = BallValve(server, int(count))
    while not rospy.is_shutdown():
        rospy.spin()