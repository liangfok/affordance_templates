#!/usr/bin/env python
import rospy
import roslib; roslib.load_manifest('marker_templates')
from TaskTemplate import *
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *

import PyKDL as kdl

PREGRASP = 0
GRASP = 1
TURN = 2
RELEASE = 3

class SmallWheel(TaskTemplate):
    def __init__(self, server, id_):
        TaskTemplate.__init__(self, server, id_, "/world", "SmallWheel")

        self._diameter = 0.2286 # this is 9in in meters

        self._graspXOffset = 0.05
        self._graspZOffset = 0.01
        self._graspPitchOffset = 0.4

        self._pregraspOffsetInitial = self._pregraspOffset = 0.1
        self._pregraspMinLimit = 0.0
        self._pregraspMaxLimit = 0.3
        self._pregraspMarkerName = "SmallWheelPreGrasp" + str(id_)
        self._preGraspHandle = None

        self._turnRotationInitial = self._turnRotation = 0.3
        self._turnMarkerName = "SmallWheelTurn" + str(id_)
        self._turnHandle = None

        self._releaseOffsetInitial = self._releaseOffset = 0.1
        self._releaseMinLimit = 0.0
        self._releaseMaxLimit = 0.3
        self._releaseMarkerName = "SmallWheelRelease" + str(id_)
        self._releaseHandle = None

        self._handAdjustment = 0.0 
        self._handAdjustmentMinLimit = -1.0
        self._handAdjustmentMaxLimit = 1.0
        self._handAdjustmentMarkerName = "SmallWheelHandAdjust" + str(id_)
        self._adjustmentHandle = None

        # self._leftPathIndex = 0
        # self._rightPathIndex = 0

        self.initROS()
        self.initMenu()
        self.extendMenu()
        self.initMarker()
        self.initHands()
        # self.setupHands()

    def initMarker(self):
        path = "package://marker_templates/resources/models/torus.stl"
        markerPose = Pose()
        markerPose.position.x = 0.5
        markerPose.position.y = 0
        markerPose.position.z = 0.03
        rot = kdl.Rotation.RPY(1.57, 0, 1.57)
        q = rot.GetQuaternion()
        markerPose.orientation.x = q[0]
        markerPose.orientation.y = q[1]
        markerPose.orientation.z = q[2]
        markerPose.orientation.w = q[3]
        offset = Pose()
        offset.position.x = offset.position.y = offset.position.z = 0
        offset.orientation.x = offset.orientation.y = offset.orientation.z = 0
        offset.orientation.w = 1
        scaleScalar = self._diameter
        scale = Vector3(scaleScalar, scaleScalar, scaleScalar)
        self.setMarkerColor(ColorRGBA(1, 1, 1, 0.5))
        self.setMesh(path, offset, scale)
        self._SmallWheelMarker = self.createMarker(self.markerCB, markerPose)
        self.attachMenuHandler(self._SmallWheelMarker)
        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

    def extendMenu(self):
        self.menu_handler.setCheckState(self.menu_handler.insert("Scale SmallWheel diameter", callback=self.enableChangeSmallWheelSize), MenuHandler.UNCHECKED)
        self._numMenuItems += 1
        # setControl = self.menu_handler.insert("Set hand goals")
        # self._numMenuItems += 1
        # self.menu_handler.setCheckState(self.menu_handler.insert( "Set PreGrasp ", parent=setControl, command_type=MenuEntry.FEEDBACK, command="", callback=self.enablePregrasp ), MenuHandler.UNCHECKED )
        # self._numMenuItems += 1
        # self.menu_handler.setCheckState(self.menu_handler.insert( "Set Turning Goal", parent=setControl, command_type=MenuEntry.FEEDBACK, command="", callback=self.enableTurn ), MenuHandler.UNCHECKED )
        # self._numMenuItems += 1
        # self.menu_handler.setCheckState(self.menu_handler.insert( "Set Release Goal", parent=setControl, command_type=MenuEntry.FEEDBACK, command="", callback=self.enableRelease ), MenuHandler.UNCHECKED )
        # self._numMenuItems += 1        
        # self.menu_handler.setCheckState(self.menu_handler.insert( "Adjust hand location on SmallWheel", parent=setControl, command_type=MenuEntry.FEEDBACK, command="", callback=self.enableHandAdjustment), MenuHandler.UNCHECKED )
        # self._numMenuItems += 1


    def markerCB(self, feedback):
        wTc_old = getFrameFromPose(self._SmallWheelMarker.pose)
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
            self._handPoseArray[RIGHT][self._rightPathIndex].poses[index] = getPoseFromFrame(cTh)
            hand.setHandPose(getPoseFromFrame(wTc_new*cTh))
            index += 1

        self._SmallWheelMarker.pose = feedback.pose
        self.updateMarkerFrame(wTc_new)
   
    def setupHands(self):
        yellow = ColorRGBA(1.0, 1.0, 0.0, 0.4)
        green = ColorRGBA(0.0, 1.0, 0.0, 0.4)
        blue = ColorRGBA(0.0, 0.0, 1.0, 0.4)
        cyan = ColorRGBA(0.0, 1.0, 1.0, 0.4)

        wTc = getFrameFromPose(self._SmallWheelMarker.pose)
        paLeft = PoseArray()
        paRight = PoseArray()

        # set offsets
        cVlg1 = kdl.Vector(0.0, 0.0, 0)
        cVrg1 = kdl.Vector(0.0, 0.0, 0.0)
        cRlg1 = kdl.Rotation.RPY(0, 0.0, self._handAdjustment)
        cRrg1 = kdl.Rotation.RPY(0.0, 0.0, -self._handAdjustment)
        # set grasp pose to get hands in correct orientation to center of SmallWheel
        cVlg2 = kdl.Vector((self._diameter*1.1/2.0)+self._graspXOffset, 0.0, -0.085+self._graspZOffset)
        cVrg2 = kdl.Vector(-(self._diameter*1.1/2.0)-self._graspXOffset, 0.0, -0.085+self._graspZOffset)
        cRlg2 = kdl.Rotation.RPY(3.14, -1.57-self._graspPitchOffset, 0)
        cRrg2 = kdl.Rotation.RPY(3.14, -1.57+self._graspPitchOffset, 0)

        lmVo = kdl.Vector(0.0, 0.0, self._pregraspOffset)
        rmVo = kdl.Vector(0.0, 0.0, -self._pregraspOffset)
        lmRo = kdl.Rotation.RPY(0.0, 0.0, 0.0)
        rmRo = kdl.Rotation.RPY(0.0, 0.0, 0.0)
        cTlh = kdl.Frame(cRlg1, cVlg1)*kdl.Frame(cRlg2, cVlg2)*kdl.Frame(lmRo, lmVo)
        cTrh = kdl.Frame(cRrg1, cVrg1)*kdl.Frame(cRrg2, cVrg2)*kdl.Frame(rmRo, rmVo)
        self.addHand(getPoseFromFrame(wTc*cTlh), "left", yellow)
        self.addHand(getPoseFromFrame(wTc*cTrh), "right", yellow)
        paLeft.poses.append(getPoseFromFrame(cTlh))
        paRight.poses.append(getPoseFromFrame(cTrh))
        
        cTlh = kdl.Frame(cRlg1, cVlg1)*kdl.Frame(cRlg2, cVlg2)
        cTrh = kdl.Frame(cRrg1, cVrg1)*kdl.Frame(cRrg2, cVrg2)
        self.addHand(getPoseFromFrame(wTc*cTlh), "left", green)
        self.addHand(getPoseFromFrame(wTc*cTrh), "right", green)
        paLeft.poses.append(getPoseFromFrame(cTlh))
        paRight.poses.append(getPoseFromFrame(cTrh))

        cVl = kdl.Vector(0.0, 0.0, 0.0)
        cVr = kdl.Vector(0.0, 0.0, 0.0)
        cRl = kdl.Rotation.RPY(0, 0.0, self._handAdjustment + self._turnRotation)
        cRr = kdl.Rotation.RPY(0.0, 0.0, self._handAdjustment + self._turnRotation)
        # multiply with hand poses to get full transform from turn goals to center of SmallWheel
        cTlh = kdl.Frame(cRl, cVl)*kdl.Frame(cRlg1, cVlg1)*kdl.Frame(cRlg2, cVlg2)
        cTrh = kdl.Frame(cRr, cVr)*kdl.Frame(cRrg1, cVrg1)*kdl.Frame(cRrg2, cVrg2)
        turnL = getPoseFromFrame(wTc*cTlh)
        turnR = getPoseFromFrame(wTc*cTrh)
        self.addHand(turnL, "left", blue)
        self.addHand(getPoseFromFrame(wTc*cTrh), "right", blue)
        paLeft.poses.append(getPoseFromFrame(cTlh))
        paRight.poses.append(getPoseFromFrame(cTrh))

        # want post-grasp to be z directions of turn hands
        cVl = kdl.Vector( 0.0, 0.0, self._releaseOffset )
        cVr = kdl.Vector(0.0, 0.0, -self._releaseOffset)        
        cRl = kdl.Rotation.RPY(0,0,0)
        cRr = kdl.Rotation.RPY(0,0,0)
        cTlh = kdl.Frame(cRl, cVl)*kdl.Frame(cRlg1, cVlg1)*kdl.Frame(cRlg2, cVlg2)*kdl.Frame(cRl, cVl)
        cTrh = kdl.Frame(cRr, cVr)*kdl.Frame(cRrg1, cVrg1)*kdl.Frame(cRrg2, cVrg2)*kdl.Frame(cRr, cVr)
        self.addHand(getPoseFromFrame(wTc*cTlh), "left", cyan)
        self.addHand(getPoseFromFrame(wTc*cTrh), "right", cyan)
        paLeft.poses.append(getPoseFromFrame(cTlh))
        paRight.poses.append(getPoseFromFrame(cTrh))

        self._handPoseArray[LEFT].append(paLeft)
        self._handPoseArray[RIGHT].append(paRight)

    def calculateTransforms(self):

        # set grasp pose for hand adjustment
        cVlg1 = kdl.Vector(0.0, 0.0, 0)
        cVrg1 = kdl.Vector(0.0, 0.0, 0.0)
        cRlg1 = kdl.Rotation.RPY(0, 0.0, self._handAdjustment)
        cRrg1 = kdl.Rotation.RPY(0.0, 0.0, -self._handAdjustment)
        # set grasp pose to get hands in correct orientation to center of SmallWheel
        cVlg2 = kdl.Vector((self._diameter*1.1/2.0)+self._graspXOffset, 0.0, -0.085+self._graspZOffset)
        cVrg2 = kdl.Vector(-(self._diameter*1.1/2.0)-self._graspXOffset, 0.0, -0.085+self._graspZOffset)
        cRlg2 = kdl.Rotation.RPY(3.14, -1.57-self._graspPitchOffset, 0)
        cRrg2 = kdl.Rotation.RPY(3.14, -1.57+self._graspPitchOffset, 0)
        # multiply together to get full transform from grasps to center of SmallWheel
        leftGraspFrame = kdl.Frame(cRlg1, cVlg1)*kdl.Frame(cRlg2, cVlg2)
        rightGraspFrame = kdl.Frame(cRrg1, cVrg1)*kdl.Frame(cRrg2, cVrg2)
        self._handPoseArray[LEFT][0].poses[GRASP] = getPoseFromFrame(leftGraspFrame)
        self._handPoseArray[RIGHT][0].poses[GRASP] = getPoseFromFrame(rightGraspFrame)

        # want pre-grasp to be diff in hand z directions
        # need to get lmToffset, cTlm
        lmVo = kdl.Vector(0.0, 0.0, self._pregraspOffset)
        rmVo = kdl.Vector(0.0, 0.0, -self._pregraspOffset)
        lmRo = kdl.Rotation.RPY(0.0, 0.0, 0.0)
        rmRo = kdl.Rotation.RPY(0.0, 0.0, 0.0)
        # multiply hand poses w/ offset with hand adjustment to get full transform from pregrasps to center of SmallWheel
        leftFrame = leftGraspFrame*kdl.Frame(lmRo, lmVo)
        rightFrame = rightGraspFrame*kdl.Frame(rmRo, rmVo)
        self._handPoseArray[LEFT][0].poses[PREGRASP] = getPoseFromFrame(leftFrame)
        self._handPoseArray[RIGHT][0].poses[PREGRASP] = getPoseFromFrame(rightFrame) 

        # set grasp pose for hand adjustment and goal rotation
        cVl = kdl.Vector(0.0, 0.0, 0.0)
        cVr = kdl.Vector(0.0, 0.0, 0.0)
        cRl = kdl.Rotation.RPY(0, 0.0, self._handAdjustment + self._turnRotation)
        cRr = kdl.Rotation.RPY(0.0, 0.0, self._handAdjustment + self._turnRotation)
        # multiply with hand poses to get full transform from turn goals to center of SmallWheel
        leftTurnFrame = kdl.Frame(cRl, cVl)*leftGraspFrame
        rightTurnFrame = kdl.Frame(cRr, cVr)*rightGraspFrame
        self._handPoseArray[LEFT][0].poses[TURN] = getPoseFromFrame(leftTurnFrame)
        self._handPoseArray[RIGHT][0].poses[TURN] = getPoseFromFrame(rightTurnFrame) 

        # want post-grasp to be z directions of turn hands
        cVl = kdl.Vector( 0.0, 0.0, self._releaseOffset )
        cVr = kdl.Vector(0.0, 0.0, -self._releaseOffset)        
        cRl = kdl.Rotation.RPY(0,0,0)
        cRr = kdl.Rotation.RPY(0,0,0)
        # multiply together to get full transform from post turn goals to center of SmallWheel
        # opposite multiplication b/c we are doing frame of turn hands to release hands
        leftFrame = leftTurnFrame*kdl.Frame(cRl, cVl)
        rightFrame = rightTurnFrame*kdl.Frame(cRr, cVr)
        self._handPoseArray[LEFT][0].poses[RELEASE] = getPoseFromFrame(leftFrame)
        self._handPoseArray[RIGHT][0].poses[RELEASE] = getPoseFromFrame(rightFrame)

    def enableChangeSmallWheelSize(self, feedback):
        handle = feedback.menu_entry_id
        state = self.menu_handler.getCheckState(handle)
        if state == MenuHandler.UNCHECKED:
            self.menu_handler.setCheckState(handle, MenuHandler.CHECKED)
            self.createScaleMarker()
        else:
            self.menu_handler.setCheckState(handle, MenuHandler.UNCHECKED)
            self.removeScaleMarker()

    def enablePregrasp(self, feedback):
        self._preGraspHandle = feedback.menu_entry_id
        state = self.menu_handler.getCheckState(self._preGraspHandle)
        if state == MenuHandler.UNCHECKED:
            self.menu_handler.setCheckState(self._preGraspHandle, MenuHandler.CHECKED)
            wTc = getFrameFromPose(self._SmallWheelMarker.pose)
            # get current position/orientation of hands based on last grasp
            poseLeft = self._hands[LEFT][GRASP].getCurrentPose()
            poseRight = self._hands[RIGHT][GRASP].getCurrentPose()
            # set pose array w/ current grasp positions
            wTlh = getFrameFromPose(poseLeft)
            wTrh = getFrameFromPose(poseRight)
            self._handPoseArray[LEFT][0].poses[GRASP] = getPoseFromFrame(wTc.Inverse()*wTlh)
            self._handPoseArray[RIGHT][0].poses[GRASP] = getPoseFromFrame(wTc.Inverse()*wTrh)
            self.calculateTransforms()
            if self._adjustmentHandle is not None:
                self.removeHandAdjustmentMarkers()
                self._adjustmentHandle = None
                self.menu_handler.setCheckState(self._adjustmentHandle, MenuHandler.UNCHECKED)
            elif self._turnHandle is not None:
                self.removeTurnGoalMarkers()
                self._turnHandle = None
                self.menu_handler.setCheckState(self._turnHandle, MenuHandler.UNCHECKED)
            elif self._releaseHandle is not None:
                self.removeReleaseMarkers()
                self._releaseHandle = None
                self.menu_handler.setCheckState(self._releaseHandle, MenuHandler.UNCHECKED)
            # create pregrasp hand markers
            color = {}
            color["r"] = 1
            color["g"] = 1
            color["b"] = 0
            color["a"] = 0.75
            # set pose of markers
            print "left relative poses are \n"
            print self._handPoseArray[LEFT]
            print "right relative poses are \n"
            print self._handPoseArray[RIGHT]
            cTlh = getFrameFromPose(self._handPoseArray[LEFT][0].poses[PREGRASP])
            cTrh = getFrameFromPose(self._handPoseArray[RIGHT][0].poses[PREGRASP])
            wTlh = wTc*cTlh
            wTrh = wTc*cTrh
            self._hands[LEFT][PREGRASP].setHandPose(getPoseFromFrame(wTlh))
            self._hands[RIGHT][PREGRASP].setHandPose(getPoseFromFrame(wTrh))
            self.createPreGraspMarker()
        else:
            self.menu_handler.setCheckState(self._preGraspHandle, MenuHandler.UNCHECKED)
            self.removePreGraspMarker()
            self.calculateTransforms()

        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

    def enableTurn(self, feedback):
        self._turnHandle = feedback.menu_entry_id
        state = self.menu_handler.getCheckState(self._turnHandle)
        if state == MenuHandler.UNCHECKED:
            self.menu_handler.setCheckState(self._turnHandle, MenuHandler.CHECKED)
            wTc = getFrameFromPose(self._SmallWheelMarker.pose)            
            # get current position/orientation of hands based on last grasp
            poseLeft = self._hands[LEFT][GRASP].getCurrentPose()
            poseRight = self._hands[RIGHT][GRASP].getCurrentPose()
            # set pose array w/ current grasp positions
            wTlh = getFrameFromPose(poseLeft)
            wTrh = getFrameFromPose(poseRight)
            self._handPoseArray[LEFT][0].poses[GRASP] = getPoseFromFrame(wTc.Inverse()*wTlh)
            self._handPoseArray[RIGHT][0].poses[GRASP] = getPoseFromFrame(wTc.Inverse()*wTrh)
            self.calculateTransforms()
            if self._adjustmentHandle is not None:
                self.removeHandAdjustmentMarkers()
                self._adjustmentHandle = None
                self.menu_handler.setCheckState(self._adjustmentHandle, MenuHandler.UNCHECKED)
            elif self._preGraspHandle is not None:
                self.removePreGraspMarker()
                self._preGraspHandle = None
                self.menu_handler.setCheckState(self._preGraspHandle, MenuHandler.UNCHECKED)
            elif self._releaseHandle is not None:
                self.removeReleaseMarkers()
                self._releaseHandle = None
                self.menu_handler.setCheckState(self._releaseHandle, MenuHandler.UNCHECKED)
            # create turn goal hand markers
            color = {}
            color["r"] = 1
            color["g"] = 0
            color["b"] = 0
            color["a"] = 0.75
            cTlh = getFrameFromPose(self._handPoseArray[LEFT][0].poses[TURN])
            cTrh = getFrameFromPose(self._handPoseArray[RIGHT][0].poses[TURN])
            wTlh = wTc*cTlh
            wTrh = wTc*cTrh
            self._hands[LEFT][TURN].setHandPose(getPoseFromFrame(wTlh))
            self._hands[RIGHT][TURN].setHandPose(getPoseFromFrame(wTrh))
            self.createTurnMarker()
            self.menu_handler.reApply(self.server)
        else:
            self.menu_handler.setCheckState(self._turnHandle, MenuHandler.UNCHECKED)
            self.removeTurnGoalMarkers()
            self.calculateTransforms()

        self.menu_handler.reApply(self.server)
        self.server.applyChanges()


    def enableRelease(self, feedback):
        self._releaseHandle = feedback.menu_entry_id
        state = self.menu_handler.getCheckState(self._releaseHandle)
        if state == MenuHandler.UNCHECKED:
            self.menu_handler.setCheckState(self._releaseHandle, MenuHandler.CHECKED)
            wTc = getFrameFromPose(self._SmallWheelMarker.pose)            
            # get current position/orientation of hands based on last grasp
            poseLeft = self._hands[LEFT][GRASP].getCurrentPose()
            poseRight = self._hands[RIGHT][GRASP].getCurrentPose()
            # set pose array w/ current grasp positions
            wTlh = getFrameFromPose(poseLeft)
            wTrh = getFrameFromPose(poseRight)
            self._handPoseArray[LEFT][0].poses[GRASP] = getPoseFromFrame(wTc.Inverse()*wTlh)
            self._handPoseArray[RIGHT][0].poses[GRASP] = getPoseFromFrame(wTc.Inverse()*wTrh)
            self.calculateTransforms()
            if self._adjustmentHandle is not None:
                self.removeHandAdjustmentMarkers()
                self._adjustmentHandle = None
                self.menu_handler.setCheckState(self._adjustmentHandle, MenuHandler.UNCHECKED)
            elif self._preGraspHandle is not None:
                self.removePreGraspMarker()
                self._preGraspHandle = None
                self.menu_handler.setCheckState(self._preGraspHandle, MenuHandler.UNCHECKED)
            elif self._turnHandle is not None:
                self.removeTurnGoalMarkers()
                self._turnHandle = None
                self.menu_handler.setCheckState(self._turnHandle, MenuHandler.UNCHECKED)
            # create turn goal hand markers
            color = {}
            color["r"] = 0
            color["g"] = 1
            color["b"] = 1
            color["a"] = 0.75
            cTlh = getFrameFromPose(self._handPoseArray[LEFT][0].poses[RELEASE])
            cTrh = getFrameFromPose(self._handPoseArray[RIGHT][0].poses[RELEASE])
            wTlh = wTc*cTlh
            wTrh = wTc*cTrh
            self._hands[LEFT][RELEASE].setHandPose(getPoseFromFrame(wTlh))
            self._hands[RIGHT][RELEASE].setHandPose(getPoseFromFrame(wTrh))     
            self.createReleaseMarker()
        else:
            self.menu_handler.setCheckState(self._releaseHandle, MenuHandler.UNCHECKED)
            self.removeReleaseMarkers()
            self.calculateTransforms()

        self.menu_handler.reApply(self.server)
        self.server.applyChanges()


    def enableHandAdjustment(self, feedback):
        self._adjustmentHandle = feedback.menu_entry_id
        state = self.menu_handler.getCheckState(self._adjustmentHandle)
        if state == MenuHandler.UNCHECKED:
            self.menu_handler.setCheckState(self._adjustmentHandle, MenuHandler.CHECKED)
            wTc = getFrameFromPose(self._SmallWheelMarker.pose)            
            # get current position/orientation of hands based on last grasp
            poseLeft = self._hands[LEFT][GRASP].getCurrentPose()
            poseRight = self._hands[RIGHT][GRASP].getCurrentPose()
            # set pose array w/ current grasp positions
            wTlh = getFrameFromPose(poseLeft)
            wTrh = getFrameFromPose(poseRight)
            self._handPoseArray[LEFT][0].poses[GRASP] = getPoseFromFrame(wTc.Inverse()*wTlh)
            self._handPoseArray[RIGHT][0].poses[GRASP] = getPoseFromFrame(wTc.Inverse()*wTrh)
            self.calculateTransforms()
            if self._releaseHandle is not None:
                self.removeReleaseMarkers()
                self._releaseHandle = None
                self.menu_handler.setCheckState(self._releaseHandle, MenuHandler.UNCHECKED)
            elif self._preGraspHandle is not None:
                self.removePreGraspMarker()
                self._preGraspHandle = None
                self.menu_handler.setCheckState(self._preGraspHandle, MenuHandler.UNCHECKED)
            elif self._turnHandle is not None:
                self.removeTurnGoalMarkers()
                self._turnHandle = None
                self.menu_handler.setCheckState(self._turnHandle, MenuHandler.UNCHECKED)
            color = {}
            color["r"] = 0
            color["g"] = 0
            color["b"] = 1
            color["a"] = 0.75
            cTlh = getFrameFromPose(self._handPoseArray[LEFT][0].poses[GRASP])
            cTrh = getFrameFromPose(self._handPoseArray[RIGHT][0].poses[GRASP])
            wTlh = wTc*cTlh
            wTrh = wTc*cTrh
            self._hands[LEFT][GRASP].setHandPose(getPoseFromFrame(wTlh))
            self._hands[RIGHT][GRASP].setHandPose(getPoseFromFrame(wTrh))   
            self.createHandAdjustmentMarker()
        else:
            self.menu_handler.setCheckState(self._adjustmentHandle, MenuHandler.UNCHECKED)
            self.removeHandAdjustmentMarkers()
            self.calculateTransforms()

        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

    def createScaleMarker(self):
        marker = InteractiveMarker()    
        marker.header.frame_id = self._frameID
        marker.name = "ScaleSmallWheelDiameter"
        wTc = getFrameFromPose(self._SmallWheelMarker.pose)
        marker.pose = getPoseFromFrame(wTc)
        self._lastX = marker.pose.position.x
        marker.controls.append(CreateTransRotControl("TranslateX"))
        self.addInteractiveMarker(marker,  self.scaleSmallWheelXY)
        self.menu_handler.reApply( self.server )
        self.server.applyChanges()

    def createPreGraspMarker(self):
        
        marker = InteractiveMarker()
        marker.header.frame_id = self._frameID
        marker.name = self._pregraspMarkerName
        # marker.scale = self._torusMeshScalar
        marker.scale = 0.9
        wTc = getFrameFromPose(self._SmallWheelMarker.pose)
        marker.pose = getPoseFromFrame(wTc)
        marker.controls.append(CreateTransRotControl("TranslateX"))
        print "pregrasp interactive marker"
        print str(marker)
        self.addInteractiveMarker(marker,  self.preGraspCallback)
        self.server.applyChanges()
        
        return marker

    def createTurnMarker(self):
        marker = InteractiveMarker()
        marker.header.frame_id = self._frameID
        marker.name = self._turnMarkerName
        # marker.scale = self._torusMeshScalar
        marker.scale = 0.9
        wTc = getFrameFromPose(self._SmallWheelMarker.pose)
        marker.pose = getPoseFromFrame(wTc)
        marker.controls.append(CreateTransRotControl("RotateZ"))
        self.addInteractiveMarker(marker,  self.turnCallback)
        self.server.applyChanges()

        return marker

    def createReleaseMarker(self):
        marker = InteractiveMarker()
        marker.header.frame_id = self._frameID
        marker.name = self._releaseMarkerName
        # marker.scale = self._torusMeshScalar
        marker.scale = 0.9
        wTc = getFrameFromPose(self._SmallWheelMarker.pose)
        # put orientation of 
        ### need to rotate this in z
        cV1 = kdl.Vector(0.0, 0.0, 0.0)
        cR1 = kdl.Rotation.RPY(0, 0, self._turnRotation)
        frame = wTc*kdl.Frame(cR1, cV1)
        marker.pose = getPoseFromFrame(frame)
        marker.controls.append(CreateTransRotControl("TranslateX"))
        self.addInteractiveMarker(marker, self.releaseCallback)
        self.server.applyChanges()
        
        return marker

    def createHandAdjustmentMarker(self):
        marker = InteractiveMarker()
        marker.header.frame_id = self._frameID
        marker.name = self._handAdjustmentMarkerName
        # marker.scale = self._torusMeshScalar
        marker.scale = 0.9
        wTc = getFrameFromPose(self._SmallWheelMarker.pose)
        marker.pose = getPoseFromFrame(wTc)
        marker.controls.append(CreateTransRotControl("TranslateY"))
        self.addInteractiveMarker(marker,  self.handAdjustmentCallback)
        self.server.applyChanges()

        return marker

    def removeScaleMarker(self):
        self.removeInteractiveMarker("ScaleSmallWheelDiameter")
        self.menu_handler.reApply( self.server )
        self.server.applyChanges()

    def removePreGraspMarker(self):
        try:
            self.removeInteractiveMarker(self._pregraspMarkerName)
            return True
        except:
            return False

    def removeTurnGoalMarkers(self):
        try:
            self.removeInteractiveMarker(self._turnMarkerName)
            return True
        except:
            return False


    def removeReleaseMarkers(self):
        try:
            self.removeInteractiveMarker(self._releaseMarkerName)
            return True
        except:
            return False


    def removeHandAdjustmentMarkers(self):
        try:
            self.removeInteractiveMarker(self._handAdjustmentMarkerName)
            return True
        except:
            return False

    def scaleSmallWheelXY(self, feedback):
        scaleXFactor = 50
        diff = feedback.pose.position.x - self._lastX
        self._lastX = feedback.pose.position.x
        marker = None
        for control in self._SmallWheelMarker.controls:
            if control.name is "visual":
                marker = control.markers[0]
        if marker is not None:
            marker.scale.x += diff * scaleXFactor
            marker.scale.y += diff * scaleXFactor
            self._diameter = marker.scale.x
        self.server.insert(self._SmallWheelMarker)
        self.calculateTransforms()        
        wTc = getFrameFromPose(self._SmallWheelMarker.pose)
        index = 0
        for hand in self._hands[LEFT]:
            cTh = getFrameFromPose(self._handPoseArray[LEFT][0].poses[index])
            index += 1
            hand.setHandPose(getPoseFromFrame(wTc*cTh))
        index = 0
        for hand in self._hands[RIGHT]:
            cTh = getFrameFromPose(self._handPoseArray[RIGHT][0].poses[index])
            index += 1
            hand.setHandPose(getPoseFromFrame(wTc*cTh))       

        self.server.applyChanges()

    def preGraspCallback(self, feedback):
        wTc = getFrameFromPose(self._SmallWheelMarker.pose)
        newMarkerPose = getPoseFromFrame(wTc.Inverse()*getFrameFromPose(feedback.pose))
        x_dist = self._pregraspOffsetInitial + newMarkerPose.position.x

        if x_dist > self._pregraspMaxLimit:
            newMarkerPose.position.x = self._pregraspMaxLimit - self._pregraspOffsetInitial
            feedback.pose = getPoseFromFrame(wTc*getFrameFromPose(newMarkerPose))
            self.server.setPose(self._pregraspMarkerName, feedback.pose)
        elif x_dist < self._pregraspMinLimit:
            newMarkerPose.position.x = self._pregraspMinLimit - self._pregraspOffsetInitial
            feedback.pose = getPoseFromFrame(wTc*getFrameFromPose(newMarkerPose))
            self.server.setPose(self._pregraspMarkerName, feedback.pose)
        
        self._pregraspOffset = x_dist
        self.calculateTransforms()
        
        cTlh = getFrameFromPose(self._handPoseArray[LEFT][0].poses[PREGRASP])
        cTrh = getFrameFromPose(self._handPoseArray[RIGHT][0].poses[PREGRASP])
        self._hands[LEFT][PREGRASP].setHandPose(getPoseFromFrame(wTc*cTlh))
        self._hands[RIGHT][PREGRASP].setHandPose(getPoseFromFrame(wTc*cTrh))

        self.server.applyChanges()

    def turnCallback(self, feedback):
        wTc = getFrameFromPose(self._SmallWheelMarker.pose)
        newMarkerPose = getPoseFromFrame(wTc.Inverse()*getFrameFromPose(feedback.pose))

        x = newMarkerPose.orientation.x
        y = newMarkerPose.orientation.y
        z = newMarkerPose.orientation.z
        w = newMarkerPose.orientation.w
        qmag = math.sqrt(x**2 + y**2 + z**2 + w**2)
        (r,p,y) = kdl.Rotation.Quaternion(newMarkerPose.orientation.x / qmag, newMarkerPose.orientation.y / qmag, newMarkerPose.orientation.z / qmag, newMarkerPose.orientation.w / qmag).GetRPY()
        self._turnRotation = self._turnRotationInitial + y

        self.calculateTransforms()
        
        cTlh = getFrameFromPose(self._handPoseArray[LEFT][0].poses[TURN])
        cTrh = getFrameFromPose(self._handPoseArray[RIGHT][0].poses[TURN])
        self._hands[LEFT][TURN].setHandPose(getPoseFromFrame(wTc*cTlh))
        self._hands[RIGHT][TURN].setHandPose(getPoseFromFrame(wTc*cTrh))

        self.server.applyChanges()

    def releaseCallback(self, feedback):
        wTc = getFrameFromPose(self._SmallWheelMarker.pose)
        newMarkerPose = getPoseFromFrame(wTc.Inverse()*getFrameFromPose(feedback.pose))
        x_dist = self._releaseOffsetInitial + newMarkerPose.position.x

        if x_dist > self._releaseMaxLimit:
            newMarkerPose.position.x = self._releaseMaxLimit - self._releaseOffsetInitial
            feedback.pose = getPoseFromFrame(wTc*getFrameFromPose(newMarkerPose))
            self.server.setPose(self._releaseMarkerName, feedback.pose)
        elif x_dist < self._releaseMinLimit:
            newMarkerPose.position.x = self._releaseMinLimit - self._releaseOffsetInitial
            feedback.pose = getPoseFromFrame(wTc*getFrameFromPose(newMarkerPose))
            self.server.setPose(self._releaseMarkerName, feedback.pose)
        
        self._releaseOffset = x_dist
        self.calculateTransforms()
        
        cTlh = getFrameFromPose(self._handPoseArray[LEFT][0].poses[RELEASE])
        cTrh = getFrameFromPose(self._handPoseArray[RIGHT][0].poses[RELEASE])
        self._hands[LEFT][RELEASE].setHandPose(getPoseFromFrame(wTc*cTlh))
        self._hands[RIGHT][RELEASE].setHandPose(getPoseFromFrame(wTc*cTrh))

        self.server.applyChanges()

    def handAdjustmentCallback(self, feedback):
        wTc = getFrameFromPose(self._SmallWheelMarker.pose)
        wTm = getFrameFromPose(feedback.pose)
        cTm = wTc.Inverse()*wTm
        newPose = getPoseFromFrame(cTm)
        y_dist = newPose.position.y

        if y_dist >= self._handAdjustmentMaxLimit :
            y_dist = newPose.position.y = self._handAdjustmentMaxLimit
            feedback.pose = getPoseFromFrame(wTc*getFrameFromPose(newPose))
            self.server.setPose(self._handAdjustmentMarkerName, feedback.pose)
        if y_dist <= self._handAdjustmentMinLimit :
            y_dist = newPose.position.y = self._handAdjustmentMinLimit
            feedback.pose = getPoseFromFrame(wTc*getFrameFromPose(newPose))
            self.server.setPose(self._handAdjustmentMarkerName, feedback.pose)

        if y_dist > 0 :
            self._handAdjustment = math.asin( (y_dist/self._handAdjustmentMaxLimit) )
        else :
            self._handAdjustment = math.asin( (y_dist/self._handAdjustmentMinLimit) )
        self._handAdjustment = math.copysign(self._handAdjustment, y_dist)

        self.calculateTransforms()

        cTlh = getFrameFromPose(self._handPoseArray[LEFT][0].poses[GRASP])
        cTrh = getFrameFromPose(self._handPoseArray[RIGHT][0].poses[GRASP])
        self._hands[LEFT][GRASP].setHandPose(getPoseFromFrame(wTc*cTlh))
        self._hands[RIGHT][GRASP].setHandPose(getPoseFromFrame(wTc*cTrh))

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
        separate_topic = sys.argv[2]
    key = "SmallWheel" + count
    rospy.init_node(key)
    if separate_topic:
        server = InteractiveMarkerServer(key)
    else:
        server = InteractiveMarkerServer("MarkerTemplates", key)
    template = SmallWheel(server, int(count))
    while not rospy.is_shutdown():
        rospy.spin()
