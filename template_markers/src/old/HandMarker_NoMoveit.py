import rospy
import PyKDL as kdl
import tf
import threading
import select
import time

from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Pose, Quaternion, Vector3
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from MarkerTemplate import MarkerTemplate
from std_msgs.msg import Float64
#from marker_templates.cfg import HandsConfig
from template_utilities import *
from nasa_robot_teleop import moveit_interface
from copy import deepcopy


class HandMarker(MarkerTemplate):
    # server = interface to interactive marker server
    # instance = number of type instance 
    # frame = frame for hand in rviz
    # hand => use HandIdentifier to determine which hand to create
    # color => color of hands
    # config_file = file that provides mesh path, offset and ready pose for robot hand 
    # (could be Valkyrie, R2, etc)
    def __init__(self, server, instance, hand, color, frame="\world", config_file=None):
        MarkerTemplate.__init__(self, server, key, id_)
        self._name = hand + "Hand"
        self._id = instance
        self._color = color
        if hand is LEFT:
            self._left = True
            self._right = False
        else:
            self._left = False
            self._right = True

        self.setup(config_file)

    @property
    def setFrameID(self, frame):
        self._frameID = frame

    def getFrameID(self):
        return self._frameID

    def setup(self, config_file):
        self._robotFrameID = '/v1/Pelvis'
        self._tfListener = tf.TransformListener()
        self._syncRotation = Pose()
        self._center = Pose()
        self._leftRobotActual = Pose()
        self._rightRobotActual = Pose()
        self._nudgeDistance = 0.01

        self.initMenu()
        self.initROS()
        # for now, we're assuming that if config file is None, is Valkyrie
        if config_file is None:
            self.initHand()
        else:
            pass

        self._hand = self.addHand(self._meshOffset, self._meshPath)
        self._previousPose = self._hand.pose

        # for printing hand positions
        self._handPositionIndex = 0    
        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

    def initMenu(self):

        self.menu_handler.insert("Sync with robot hands", callback=self.syncActualPoses)
        self.menu_handler.insert("Publish", callback=self.publishHand)
        self.menu_handler.insert("Go to Ready Pose", callback=self.publishReadyPose)
        self.menu_handler.setCheckState(self.menu_handler.insert("Show hands", callback=self.enableShowHands), MenuHandler.CHECKED)
        self.menu_handler.setCheckState(self.menu_handler.insert("Show hand controls", callback=self.enableShowHandControls), MenuHandler.CHECKED)
        self.initNudgeMenu()

    def initROS(self):
        topic = "/" + self._hand + "_hand_goal/marker"
        self._handPub = rospy.Publisher(topic, Marker)

    def initHand(self):
        # setup initial poses
        self._readyPose = Pose()
        self._readyPose.position.x = 0.18
        self._readyPose.position.y = 0.39
        if self._right:
            self._readyPose.position.y *= -1
        self._readyPose.position.z = 1.38
        self._readyPose.orientation.x = -0.707
        self._readyPose.orientation.y = 0
        self._readyPose.orientation.z = 0
        self._readyPose.orientation.w = 0.707

        if self._left:
            self._meshPath = "package://marker_templates/resources/models/Hands/LeftHand.stl"
            # rotation for the visual model to align with robot
            q = kdl.Rotation.RPY(-1.57, 0, -1.57).GetQuaternion()
        else:
            self._meshPath = "package://marker_templates/resources/models/Hands/RightHand.stl"
            # rotation for the visual model to align with robot
            q = kdl.Rotation.RPY(1.57, 0, -1.57).GetQuaternion()
        # add hand
        self._meshOffset = Quaternion()
        self._meshOffset.x = q[0]
        self._meshOffset.y = q[1]
        self._meshOffset.z = q[2]
        self._meshOffset.w = q[3]

    def addHand(self, offset_quat, meshPath):
        name = self._name + self._id
        # create IM in 'world' frame with name and scale factor of 0.15
        marker = CreateInteractiveMarker(self._frameID, name, 0.15)
        # create visual hand marker with scale of 0.001 and id of 0
        mesh = CreateMeshMarker(0.001, meshPath, 0)
        
        mesh.color = self._color
        # adjust orientation to align hand mesh to actual hand coordinate system
        mesh.pose.orientation = offset_quat
        # create a control out of the hand visual marker and append it to the interactive_marker
        marker.controls.append(CreateVisualControlFromMarker(mesh))
        # attach 6 dof controls to the interactive marker
        marker.controls.extend(Create6DOFControls())
        # set pose for InteractiveMarkers on the server so we don't create it at the origin
        q = kdl.Rotation.RPY(-1.57, 0, 0).GetQuaternion()
        if self._right:
            limb = -1
        else:
            limb = 1
        offset = Pose()
        offset.position.x = 0.5
        offset.position.y = 0.25 * limb
        offset.position.z = 0
        offset.orientation.x = q[0]
        offset.orientation.y = q[1]
        offset.orientation.z = q[2]
        offset.orientation.w = q[3]
        # # get the offset relative to the robot in the world frame
        # startPose = getPoseFromRobotFrame(self._tfListener, self._robotFrameID, offset)
        # if startPose == None:
        #     # unable to get relative pose (TF not available? sim not running?), just use the initial offset
        #     startPose = offset
        # startPose.position.z = 1
        marker.pose = offset

        # add interactive_marker to template and attach menu handler
        self.addInteractiveMarker(marker, self.processHandFeedback)
        self.attachMenuHandler(marker)

        return marker

    def syncActualPoses(self, feedback):
        robotActual = Pose()

        if self._left:
            frame = '/v1/LeftPalm'
        else:
            frame = '/v1/RightPalm'

        no_transform = True
        # get this position in world frame
        while no_transform:
            try:
                t = rospy.Time(0)
                self._tfListener.waitForTransform(frame, self._frameID, t, rospy.Duration(2))
                (pos, rot) = self._tfListener.lookupTransform(self._frameID, frame, t)
                no_transform = False
            except (tf.ConnectivityException, tf.LookupException, tf.ExtrapolationException):
                continue

        robotActual.position.x = pos[0]
        robotActual.position.y = pos[1]
        robotActual.position.z = pos[2]
        robotActual.orientation.x = rot[0]
        robotActual.orientation.y = rot[1]
        robotActual.orientation.z = rot[2]
        robotActual.orientation.w = rot[3]

        pose = self.server.get(self._hand.name).pose
        self._previousPose = self._hand.pose
        self._hand.pose = robotActual
        self.server.setPose(self._hand.name, self._hand.pose)
        self.server.applyChanges()

    def nudge(self, distance, axis, direction):
        pose = self._previousPose = self._hand.pose
        if axis == "X":
            if direction == "+":
                pose.position.x += distance
            else:
                pose.position.x -= distance
        elif axis == "Y":
            if direction == "+":
                pose.position.y += distance
            else:
                pose.position.y -= distance
        if axis == "Z":
            if direction == "+":
                pose.position.z += distance
            else:
                pose.position.z -= distance
        self.setHandPose(pose)

    def enableShowHands(self, feedback):
        handle = feedback.menu_entry_id
        state = self.menu_handler.getCheckState(handle)
        if state is MenuHandler.UNCHECKED:
            self.menu_handler.setCheckState(handle, MenuHandler.CHECKED)
            self.showHands()
        else:
            self.menu_handler.setCheckState(handle, MenuHandler.UNCHECKED)
            self.hideHands()

        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

    def enableShowHandControls(self, feedback):
        handle = feedback.menu_entry_id
        state = self.menu_handler.getCheckState(handle)
        if state is MenuHandler.UNCHECKED:
            self.menu_handler.setCheckState(handle, MenuHandler.CHECKED)
            self.showHandControls()
        else:
            self.menu_handler.setCheckState(handle, MenuHandler.UNCHECKED)
            self.hideHandControls()

        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

    def showHandControls(self):
        marker = self.getMarker(self._hand.name)
        control = marker.controls[0]
        mesh = control.markers[0]
        mesh.type = Marker.MESH_RESOURCE
        mesh.mesh_use_embedded_materials = True
        mesh.scale = Vector3(0.001, 0.001, 0.001)
        mesh.mesh_resource = self._meshPath
        control.markers = []
        control.markers.append(mesh)
        marker.controls = []
        marker.controls.append(control)
        marker.controls.extend(Create6DOFControls())
        self.server.insert(marker)
        self.server.applyChanges()

    def hideHandControls(self):
        marker = self.getMarker(self._hand.name)
        marker.controls[:] = [control for control in marker.controls if control.name is "visual"]
        self.server.insert(marker)
        self.attachMenuHandler(marker)
        self.menu_handler.reApply(self.server)
        self.server.applyChanges()
    
    def showHands(self):
        marker = self.getMarker(self._hand.name)
        control = marker.controls[0]
        mesh = control.markers[0]
        mesh.type = Marker.MESH_RESOURCE
        mesh.mesh_use_embedded_materials = True
        mesh.scale = Vector3(0.001, 0.001, 0.001)
        mesh.mesh_resource = self._meshPath
        control.markers = []
        control.markers.append(mesh)
        marker.controls = []
        marker.controls.append(control)
        self.server.insert(marker)
        self.server.applyChanges()

    def hideHands(self):
        marker = self.getMarker(self._hand.name)
        i = 0
        for c in marker.controls:
            if c.name is "visual":
                control = c
                mesh = c.markers[0]
                index  = i
            i += 1
                    
        mesh.type = Marker.SPHERE
        mesh.mesh_use_embedded_materials = False
        mesh.mesh_resource = ""
        mesh.scale = Vector3(0.05,0.05,0.05)
        control.markers[0] = mesh
        marker.controls[index] = control
        self.server.insert(marker)
        self.attachMenuHandler(marker)
        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

    def printHandPoses(self, feedback):
        pose = self._hand.pose
        with open(self._hand + "HandPosConfig.txt", "a") as f:
            f.write("index " + str(self._handPositionIndex) + "\n")
            f.write(str(pose) + "\n")
            self._handPositionIndex += 1
        print "HANDS: " + self._hand + " hand pose in " + self._frameID + " is " + str(pose)

    def processHandFeedback(self, feedback):
        self._hand.pose = feedback.pose

    def publishHand(self, feedback):
        marker = self._hand
        visual = GetVisualMarker(marker)
        visual.header.frame_id = self._frameID
        visual.pose = marker.pose
        self._handPub.publish(visual)

    def publishReadyPose(self, feedback):
        """Publishes hands to Ready Pose"""
        marker = Marker()
        marker.pose = deepcopy(self._readyPose)

        self._handPub.publish(marker)
        self._hand.pose = marker.pose
        self.server.setPose(self._hand.name, self._hand.pose)
        self.server.applyChanges()

        self.server.applyChanges()

    def nudgeX(self, feedback):
        self.nudge(self._nudgeDistance, "X", "+")

    def nudgeXNeg(self, feedback):
        self.nudge(self._nudgeDistance, "X", "-")

    def nudgeY(self, feedback):
        self.nudge(self._nudgeDistance, "Y", "+")

    def nudgeYNeg(self, feedback):
        self.nudge(self._nudgeDistance, "Y", "-")

    def nudgeZ(self, feedback):
        self.nudge(self._nudgeDistance, "Z", "+")

    def nudgeZNeg(self, feedback):
        self.nudge(self._nudgeDistance, "Z", "-")

    def getCurrentPose(self):
        return self._hand.pose

    def setHandPose(self, pose):
        self._hand.pose = pose
        self.server.insert(self._hand)
        self.server.applyChanges()

    def getName(self):
        return self._hand.name

    def setControls(self, controls):
        marker = self.getMarker(self._hand.name)
        for control in controls:
            marker.controls.append(CreateTransRotControl(control))
        self.server.insert(marker)
        
        self.server.applyChanges()

    def removeControls(self, controls):
        marker = self.getMarker(self._hand.name)
        for name in controls:
            marker.controls[:] = [control for control in marker.controls if control.name is not name]
        self.server.insert(marker)
       
        self.server.applyChanges()
