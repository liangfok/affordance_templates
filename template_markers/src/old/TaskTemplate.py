#!/usr/bin/env python
import rospy

import tf
import os
import PyKDL as kdl

from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from nav_msgs.msg import Path
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from HandMarker import *
from template_utilities import *
from std_msgs.msg import ColorRGBA
from template_utilities import *
from StreamThread import *

LEFT = "left"
RIGHT = "right"
# this is for a specific task marker template that uses hands
class TaskTemplate(MarkerTemplate):

    def __init__(self, server, id_, name, frame):
        MarkerTemplate.__init__(self, server, id_, name)
        self._id = id_
        self._frameID = frame
        self._name = name
        self._handMeshPath = None
        self._handMeshPose = None
        self._handMeshScale = None
        # self._hands holds HandMarker objects
        self._hands = {}
        self._hands[LEFT] = []
        self._hands[RIGHT] = []
        # self._feet  holds FootMarker objects
        self._feet = {}
        self._feet[LEFT] = []
        self._feet[RIGHT] = []

        # self._handFiles holds files for hand paths?
        self._handFiles = {}
        self._handFiles[LEFT] = []
        self._handFiles[RIGHT] = []
        self._handFileIndices = {}
        self._handFileIndices[LEFT] = []
        self._handFileIndices[RIGHT] = []
        # self._handPoseArray holds pose array to send out for hands
        self._handPoseArray = {}
        self._handPoseArray[LEFT] = []
        self._handPoseArray[RIGHT] = []

        self._numMenuItems = 0
        self._numPtsPerSeq = 1
        self._leftHandPathIndex = -1
        self._rightHandPathIndex = -1

        self._tfListener = tf.TransformListener()
        self._streamThread = None

#######################################################
################INIT ROS###############################
#######################################################
    def initROS(self):
        self._leftHandPoseArrayPub = rospy.Publisher("/left_hand_goal/pose_array", PoseArray)
        self._leftHandPathPub = rospy.Publisher("/left_hand_goal/path", Path)
        self._rightHandPoseArrayPub = rospy.Publisher("/right_hand_goal/pose_array", PoseArray)
        self._rightHandPathPub = rospy.Publisher("/right_hand_goal/path", Path)

#######################################################
##PROPERTIES THAT MUST BE SET TO DO THINGS ############
#######################################################
    def setMarkerColor(self, color):
        self._color = color

    def setMesh(self, path, offset, scale):
        self._handMeshPath = path
        self._handMeshPose = offset
        self._handMeshScale = scale

#######################################################
########### MARKER STUFF###############################
#######################################################
    def createMarker(self, callback, offset = None):
        if self._handMeshScale is not None and self._handMeshPose is not None and self._handMeshPath is not None:
            scale = 1.0
            if offset is None:
                offset = Pose()
                offset.position.x = 1.0
                offset.position.y = 0.0
                offset.position.z = 0.0
                rot = kdl.Rotation.RPY(0, 0, 3.14)
                q = rot.GetQuaternion()
                offset.orientation.x = q[0]
                offset.orientation.y = q[1]
                offset.orientation.z = q[2]
                offset.orientation.w = q[3]
            marker = CreateInteractiveMarker(self._frameID, self._name + str(self._id), 0.5)
            # get the offset relative to the robot in the world frame
            starting_pose = getPoseFromRobotFrame(self._tfListener, "v1/Pelvis", offset)
            if starting_pose == None:
                marker.pose = offset
            else:
                marker.pose = starting_pose

            mesh = CreateMeshMarker(scale, self._handMeshPath, self._id)
            mesh.color = self._color
            mesh.pose = self._handMeshPose
            mesh.scale = self._handMeshScale
            visual = CreateVisualControlFromMarker(mesh)
            marker.controls.append(visual)
            marker.controls.extend(Create6DOFControls())
            self.addInteractiveMarker(marker, callback)
            self.menu_handler.reApply( self.server )
            self.server.applyChanges()
            return marker
        else:
            scale = 1.0
            offset = Pose()
            offset.position.x = 1.0
            offset.position.y = 0.0
            offset.position.z = 0.0
            rot = kdl.Rotation.RPY(0, 0, 3.14)
            q = rot.GetQuaternion()
            offset.orientation.x = q[0]
            offset.orientation.y = q[1]
            offset.orientation.z = q[2]
            offset.orientation.w = q[3]
            marker = CreateInteractiveMarker(self._frameID, self._name + str(self._id), 0.5)
            # get the offset relative to the robot in the world frame
            starting_pose = getPoseFromRobotFrame(self._tfListener, "v1/Pelvis", offset)
            if starting_pose == None:
                marker.pose = offset
            else:
                marker.pose = starting_pose

            marker.controls.extend(Create6DOFControls())
            self.addInteractiveMarker(marker, callback)
            self.menu_handler.reApply( self.server )
            self.server.applyChanges()
            return marker

    def enableHideMarkerControls(self, feedback):
        self._hideHandle = feedback.menu_entry_id
        state = self.menu_handler.getCheckState(self._hideHandle)
        if state is MenuHandler.UNCHECKED:
            self.menu_handler.setCheckState(self._hideHandle, MenuHandler.CHECKED)
            self.hideMarkerControls()
        else:
            self.menu_handler.setCheckState(self._hideHandle, MenuHandler.UNCHECKED)
            self.showMarkerControls()

        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

    def hideMarkerControls(self):
        # #marker = self.getMarker(self._name + str(self._id))
        marker = self.getMarker()
        marker.controls[:] = [control for control in marker.controls if (control.name == 'visual')]

    def showMarkerControls(self):
        #marker = self.getMarker(self._name + str(self._id))
        marker = self.getMarker()
        marker.controls.extend(Create6DOFControls())
        self.attachMenuHandler(marker)
        self.server.insert(marker)
        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

###################################
########## MENU STUFF #############
###################################

    def initMenu(self):
        self._publishHandle = self.menu_handler.insert("Publish hand waypoints", callback=self.publishWaypoints)
        self._numMenuItems += 1
        self._pathHandle = self._pathMenuControl = self.menu_handler.insert("Select path")
        self._numMenuItems += 1
        self._publishActualHandsHandle = self.menu_handler.insert("Publish actual hand poses", callback=self.publishActualHandPoses)
        self._streamHandle = self.menu_handler.insert("Stream Hand Positions", callback=self.enableStreaming)
        self._numMenuItems += 1
        self.menu_handler.setCheckState(self._streamHandle, MenuHandler.UNCHECKED)
        self._numMenuItems += 1
        self._savePathHandle = self.menu_handler.insert("Save current path", callback=self.saveCurrentPath)
        self._numMenuItems += 1
        hideControl = self.menu_handler.insert("Show/hide")
        self._numMenuItems += 1
        self._hideHandle = self.menu_handler.insert("Hide marker controls", parent=hideControl, command_type=MenuEntry.FEEDBACK, command="", callback=self.enableHideMarkerControls)
        self.menu_handler.setCheckState(self._hideHandle, MenuHandler.UNCHECKED)
        self._numMenuItems += 1
        self._hideHandControls = self.menu_handler.insert("Hide hand controls", parent=hideControl, command_type=MenuEntry.FEEDBACK, command="", callback=self.enableHideHandControls)
        self.menu_handler.setCheckState(self._hideHandControls, MenuHandler.UNCHECKED)
        self._numMenuItems += 1
        self._hideHandsHandle = self.menu_handler.insert("Hide hands", parent=hideControl, command_type=MenuEntry.FEEDBACK, command="", callback=self.enableHideHands)
        self.menu_handler.setCheckState(self._hideHandsHandle, MenuHandler.UNCHECKED)
        self._numMenuItems += 1
        self._numMenuItems +=1
        self._addHandsHandle = self.menu_handler.insert("Add hands")
        self._numMenuItems +=1
        self.menu_handler.insert("Add left hand", parent=self._addHandsHandle, command_type=MenuEntry.FEEDBACK, command="", callback=self.addLeftHand)
        self._numMenuItems += 1
        self.menu_handler.insert("Add right hand", parent=self._addHandsHandle, command_type=MenuEntry.FEEDBACK, command="", callback=self.addRightHand)
        self._numMenuItems += 1
        self.menu_handler.insert("Add both hands", parent=self._addHandsHandle, command_type=MenuEntry.FEEDBACK, command="", callback=self.addBothHands)
        self._numMenuItems += 1
        self.menu_handler.insert("Sync first hands to actual", callback=self.syncHandsWithActual)
        self._numMenuItems += 1
################################################
## Streaming and actual hand stuff #############
################################################

    def enableStreaming(self, feedback):
        handle = feedback.menu_entry_id
        state = self.menu_handler.getCheckState(handle)
        if state is MenuHandler.UNCHECKED:
            self.menu_handler.setCheckState(handle, MenuHandler.CHECKED)
            self.streamHandPositions()
        else:
            self.menu_handler.setCheckState(handle, MenuHandler.UNCHECKED)
            self.stopStreaming()

        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

    def streamHandPositions(self):
        #marker = self.getMarker(self._name + str(self._id))
        marker = self.getMarker()
        wTc = getFrameFromPose(marker.pose)
        # create thread
        self._streamThread = StreamThread(wTc, self._frameID, self._name)
        # start thread
        self._streamThread.start()

    def stopStreaming(self):
        # set thread streaming bool to False
        self._streamThread._stopStreaming = True
        # set thread to None
        self._streamThread = None

    def updateMarkerFrame(self, frame):
        if self._streamThread is not None:
            self._streamThread.setMarkerFrame(frame)

    def publishActualHandPoses(self, feedback):
        if self._streamThread is None:
            #marker = self.getMarker(self._name + str(self._id))
            marker = self.getMarker()
            wTc = getFrameFromPose(marker.pose)
            self._streamThread = StreamThread(wTc, self._frameID, self._name)
        self._streamThread.publish()


###################################
########## HANDS STUFF ############
###################################
    def initHands(self):
        # find all files in path (/markerTemplates/resources/paths/name)
        pathDir = roslib.packages.get_pkg_dir('marker_templates')
        hand = None
        try:
            os.chdir(pathDir + "/resources/paths/" + self._name)
            dirList = os.listdir(".")
            dirList.sort()
            for f in dirList:
                if f.endswith(".path"):
                    if LEFT in f:
                        hand = LEFT
                    else:
                        hand = RIGHT
                    self._handFiles[hand].append(f.replace(".path", ""))
                    pose = readPoseArrayFromFile(f)
                    if pose is not None:
                        self._handPoseArray[hand].append(pose)

            index = 0
            for f in self._handFiles[LEFT]:
                handle = self.menu_handler.insert(f, parent=self._pathMenuControl, command_type=MenuEntry.FEEDBACK, command="", callback=self.selectPathCB)
                self.menu_handler.setCheckState(handle, MenuHandler.UNCHECKED)
                self._handFileIndices[LEFT].append((self._numMenuItems, index))
                self._numMenuItems += 1
                index += 1
            index = 0

            for f in self._handFiles[RIGHT]:
                handle = self.menu_handler.insert(f, parent=self._pathMenuControl, command_type=MenuEntry.FEEDBACK, command="", callback=self.selectPathCB)
                self.menu_handler.setCheckState(handle, MenuHandler.UNCHECKED)
                self._handFileIndices[RIGHT].append((self._numMenuItems, index))
                self._numMenuItems += 1
                index += 1


            self.menu_handler.reApply(self.server)
            self.server.applyChanges()
        except:
            print "Could not find file directory for " + self._name


    def selectPathCB(self, feedback):
        #marker = self.getMarker(self._name + str(self._id))
        marker = self.getMarker()
        pathIndex = -1
        color = ColorRGBA(0, 1, 0, 0.4)
        try:
            self.menu_handler.setCheckState(self._pathSelectedIndex, MenuHandler.UNCHECKED)
        except:
            pass
        self._pathSelectedIndex = feedback.menu_entry_id
        for fSet in self._handFileIndices[LEFT]:
            if fSet[0] == self._pathSelectedIndex:
                if len(self._hands) > 0:
                    for hand in self._hands[LEFT]:
                        self.removeInteractiveMarker(hand.getName())
                    self._hands[LEFT] = []
                self._leftHandPathIndex = pathIndex = fSet[1]
                hand = LEFT

        for fSet in self._handFileIndices[RIGHT]:
            if fSet[0] == self._pathSelectedIndex:
                if len(self._hands[RIGHT]) > 0:
                    for hand in self._hands[RIGHT]:
                        self.removeInteractiveMarker(hand.getName())
                    self._hands[RIGHT] = []
                self._rightHandPathIndex = pathIndex = fSet[1]
                hand = RIGHT

        # try:
        wTc = getFrameFromPose(marker.pose)
        poseArray = self._handPoseArray[hand][pathIndex]
        frame = poseArray.header.frame_id
        index = 0
        for pose in poseArray.poses:
            if (frame == self._name) or (frame == ""):
                cTh = getFrameFromPose(pose)
                newPose = getPoseFromFrame(wTc*cTh)
            else:
                try:
                    t = rospy.Time(0)
                    try:
                        self._tfListener.waitForTransform(frame, "/world", t, rospy.Duration(2))
                    except:
                        print "Could not get tf transform from " + frame + " to world"
                        return
                    ps = PoseStamped()
                    ps.header.frame_id = frame
                    ps.header.seq = 0
                    ps.header.stamp = rospy.Time.now()
                    ps.pose = pose
                    try:
                        newPs = self._tfListener.transformPose("/world", ps)
                        newPose = newPs.pose
                    except:
                        print "Could not transform pose \n" + str(ps)
                        return
                except:
                    pass
            self.createHand(hand, color, index)
            self._hands[hand][index].setHandPose(newPose)
            # if hand is LEFT:
            #     parent = self._leftHandHandle
            #     callback = self.enableLeftHandOnOff
            # else:
            #     parent = self._rightHandHandle
            #     callback = self.enableRightHandOnOff

            # self.menu_handler.setCheckState(self.menu_handler.insert(hand + str(index), parent=parent, command_type=MenuEntry.FEEDBACK, command="", callback=callback), MenuHandler.CHECKED)
            # self._numMenuItems += 1
            index += 1
        # except:
        #     print "Selected path does not exist"
        self.menu_handler.setCheckState(self._pathSelectedIndex, MenuHandler.CHECKED)
        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

    def createHand(self, hand, color, id_):
        hand = HandMarker(self.server, self._name, hand, color)
        if hand is not None:
            self._hands[hand] = hand


    def syncHandsWithActual(self, feedback):
        try:
            self._hands[LEFT][0].syncActualPoses(feedback)
        except:
            pass
        try:
            self._hands[RIGHT][0].syncActualPoses(feedback)
        except:
            pass

    # adds hand to hand array with given offset
    # DOES NOT PUT pose relative to marker in self._handPoseArray
    def addHand(self, offset, hand, color):
        print "TaskTemplate.addHand: hand is " + str(hand)
        try:
            index = len(self._hands[hand])
        except:
            index = 0
        print "TaskTemplate.addHand: adding hand " + hand
        self.createHand(hand, color, index)
        self._hands[hand][index].setHandPose(offset)

    # adds hand to hand array w/ nominal offset
    # Puts pose relative to marker into self._handPoseArray
    def addNewHand(self, hand):
        color = ColorRGBA(0.0, 1.0, 0.0, 0.4)
        # marker = self.getMarker(self._name + str(self._id))
        marker = self.getMarker()
        wTc = getFrameFromPose(marker.pose)
        if hand is LEFT:
            limb = -1 #since we start the markers yawed 180, left of marker is now negative
        else:
            limb = 1
        offset = Pose()
        offset.position.x = 0
        offset.position.y = limb*0.25
        offset.position.z = 0
        q = kdl.Rotation.RPY(-1.57, 0, 3.14).GetQuaternion()
        offset.orientation.x = q[0]
        offset.orientation.y = q[1]
        offset.orientation.z = q[2]
        offset.orientation.w = q[3]
        cTh = getFrameFromPose(offset)
        pose = getPoseFromFrame(wTc*cTh)
        self.addHand(pose, hand, color)
        if self._leftHandPathIndex == -1:
            self._leftHandPathIndex = 0
            pathIndex = 0
        else:
            self._rightHandPathIndex = 0
            pathIndex = 0
        if len(self._handPoseArray[hand]) == 0:
            poseArray = PoseArray()
            poseArray.poses.append(getPoseFromFrame(cTh))
            self._handPoseArray[hand].append(poseArray)
        else:
            self._handPoseArray[hand][pathIndex].poses.append(getPoseFromFrame(cTh))

    def addBothHands(self, feedback):
        self.addLeftHand(feedback)
        self.addRightHand(feedback)

    def addLeftHand(self, feedback):
        self.addNewHand(LEFT)

    def addRightHand(self, feedback):
        self.addNewHand(RIGHT)

    def enableHideHandControls(self, feedback):
        handle = feedback.menu_entry_id
        state = self.menu_handler.getCheckState(handle)
        if state is MenuHandler.UNCHECKED:
            self.menu_handler.setCheckState(handle, MenuHandler.CHECKED)
            for hand in self._hands[LEFT]:
                hand.hideHandControls()
            for hand in self._hands[RIGHT]:
                hand.hideHandControls()
        else:
            self.menu_handler.setCheckState(handle, MenuHandler.UNCHECKED)
            for hand in self._hands[LEFT]:
                hand.showHandControls()
            for hand in self._hands[RIGHT]:
                hand.showHandControls()

        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

    def enableHideHands(self, feedback):
        handle = feedback.menu_entry_id
        state = self.menu_handler.getCheckState(handle)
        if state is MenuHandler.UNCHECKED:
            self.menu_handler.setCheckState(handle, MenuHandler.CHECKED)
            for hand in self._hands[LEFT]:
                hand.hideHands()
            for hand in self._hands[RIGHT]:
                hand.hideHands()
        else:
            self.menu_handler.setCheckState(handle, MenuHandler.UNCHECKED)
            for hand in self._hands[LEFT]:
                hand.showHands()
            for hand in self._hands[RIGHT]:
                hand.showHands()

        self.menu_handler.reApply(self.server)
        self.server.applyChanges()


###################################
#######    WAYPOINTS   ############
###################################

    def publishWaypoints(self, feedback):
        #marker = self.getMarker(self._name + str(self._id))
        marker = self.getMarker()
        wTc = getFrameFromPose(marker.pose)

        if self._leftHandPathIndex > -1:
            # first check if the hand markers have moved at all, and add those to
            # subsampled pose array
            frame = self._handPoseArray[LEFT][self._leftHandPathIndex].header.frame_id
            self._handPoseArray[LEFT][self._leftHandPathIndex].poses = []
            index = 0
            for hand in self._hands[LEFT]:
                pose = hand.getCurrentPose()
                if (frame == self._name) or (frame == ""):
                    wTh = getFrameFromPose(pose)
                    newPose = getPoseFromFrame(wTc.Inverse()*wTh)
                else:
                    try:
                        t = rospy.Time(0)
                        self._tfListener.waitForTransform(frame, "/world", t, rospy.Duration(2))
                        ps = PoseStamped()
                        ps.header.frame_id = frame
                        ps.header.seq = 0
                        ps.header.stamp = rospy.Time.now()
                        ps.pose = pose
                        newPs = self._tfListener.transformPose("/world", ps)
                        newPose = newPs.pose

                    except:
                        pass
                self._handPoseArray[LEFT][self._leftHandPathIndex].poses.append(newPose)
                index += 1

            (retPath, retPA) = processPath(self._handPoseArray[LEFT][self._leftHandPathIndex], self._numPtsPerSeq)

            poseArray = PoseArray()
            for pose in self._handPoseArray[LEFT][self._leftHandPathIndex].poses:
                if (frame == self._name) or (frame == ""):
                    cTh = getFrameFromPose(pose)
                    newPose = getPoseFromFrame(wTc*cTh)
                else:
                    try:
                        t = rospy.Time(0)
                        self._tfListener.waitForTransform(frame, "/world", t, rospy.Duration(2))
                        ps = PoseStamped()
                        ps.header.frame_id = frame
                        ps.header.seq = 0
                        ps.header.stamp = rospy.Time.now()
                        ps.pose = pose
                        newPs = self._tfListener.transformPose("/world", ps)
                        newPose = newPs.pose
                    except:
                        pass
                poseArray.poses.append(newPose)
            path = Path()
            for pose in retPath.poses:
                if (frame == self._name) or (frame == ""):
                    cTh = getFrameFromPose(pose.pose)
                    newPose = getPoseFromFrame(wTc*cTh)
                    pose.pose = newPose
                else:
                    try:
                        t = rospy.Time(0)
                        self._tfListener.waitForTransform(frame, "/world", t, rospy.Duration(2))
                        newPose = self._tfListener.transformPose("/world", ps)
                        pose = newPose
                    except:
                        pass
                path.poses.append(pose)
            poseArray.header.frame_id = path.header.frame_id = "/world"
            poseArray.header.seq = path.header.seq = self._leftHandPathIndex
            poseArray.header.stamp = path.header.stamp = rospy.Time.now()

            self._leftHandPoseArrayPub.publish(poseArray)
            self._leftHandPathPub.publish(path)

        if self._rightHandPathIndex > -1:
            # first check if the hand markers have moved at all, and add those to
            # subsampled pose array
            frame = self._handPoseArray[RIGHT][self._rightHandPathIndex].header.frame_id
            self._handPoseArray[RIGHT][self._rightHandPathIndex].poses = []
            index = 0
            for hand in self._hands[RIGHT]:
                pose = hand.getCurrentPose()
                if (frame == self._name) or (frame == ""):
                    wTh = getFrameFromPose(pose)
                    newPose = getPoseFromFrame(wTc.Inverse()*wTh)
                else:
                    try:
                        t = rospy.Time(0)
                        self._tfListener.waitForTransform(frame, "/world", t, rospy.Duration(2))
                        ps = PoseStamped()
                        ps.header.frame_id = frame
                        ps.header.seq = 0
                        ps.header.stamp = rospy.Time.now()
                        ps.pose = pose
                        newPs = self._tfListener.transformPose("/world", ps)
                        newPose = newPs.pose
                    except:
                        pass
                self._handPoseArray[RIGHT][self._rightHandPathIndex].poses.append(newPose)
                index += 1

            (retPath, retPA) = processPath(self._handPoseArray[RIGHT][self._rightHandPathIndex], self._numPtsPerSeq)

            poseArray = PoseArray()
            for pose in self._handPoseArray[RIGHT][self._rightHandPathIndex].poses:
                if (frame == self._name) or (frame == ""):
                    cTh = getFrameFromPose(pose)
                    newPose = getPoseFromFrame(wTc*cTh)
                else:
                    try:
                        t = rospy.Time(0)
                        self._tfListener.waitForTransform(frame, "/world", t, rospy.Duration(2))
                        ps = PoseStamped()
                        ps.header.frame_id = frame
                        ps.header.seq = 0
                        ps.header.stamp = rospy.Time.now()
                        ps.pose = pose
                        newPs = self._tfListener.transformPose("/world", ps)
                        newPose = newPs.pose
                    except:
                        pass
                poseArray.poses.append(newPose)
            path = Path()
            for pose in retPath.poses:
                if (frame == self._name) or (frame == ""):
                    cTh = getFrameFromPose(pose.pose)
                    newPose = getPoseFromFrame(wTc*cTh)
                    pose.pose = newPose
                else:
                    try:
                        t = rospy.Time(0)
                        self._tfListener.waitForTransform(frame, "/world", t, rospy.Duration(2))
                        newPose = self._tfListener.transformPose("/world", ps)
                        pose = newPose
                    except:
                        pass
                path.poses.append(pose)
            poseArray.header.frame_id = path.header.frame_id = "/world"
            poseArray.header.seq = path.header.seq = self._rightHandPathIndex
            poseArray.header.stamp = path.header.stamp = rospy.Time.now()

            self._rightHandPoseArrayPub.publish(poseArray)
            self._rightHandPathPub.publish(path)

    def saveCurrentPath(self, feedback):
        if self._leftHandPathIndex > -1:
            if len(self._handFiles[LEFT]) <= 0:
                fileName = "leftGrasp" + str(self._leftHandPathIndex) + ".path"
            else:
                fileName = self._handFiles[LEFT][self._leftHandPathIndex] + ".path"
            writePoseArrayToFile(fileName, self._handPoseArray[LEFT][self._leftHandPathIndex])
        elif self._rightHandPathIndex > -1:
            if len(self._handFiles[RIGHT]) <= 0:
                fileName = "rightGrasp" + str(self._rightHandPathIndex) + ".path"
            else:
                fileName = self._handFiles[RIGHT][self._rightHandPathIndex] + ".path"
            writePoseArrayToFile(fileName, self._handPoseArray[RIGHT][self._rightHandPathIndex])

