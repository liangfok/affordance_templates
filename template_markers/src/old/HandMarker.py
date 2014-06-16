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
from copy import deepcopy
from nasa_robot_teleop.moveit_interface import MoveItInterface

R2 = "r2"
V1 = "v1"
ENDEFFECTOR = "endeffector"
MANIPULATOR = "manipulator"
OPEN = "open"
CLOSED = "closed"
TRIGGER = "trigger"
THUMB_CLOSED = "thumb_closed"
LEFT = "left"
RIGHT = "right"

class HandMarker(MarkerTemplate):
    # server = interface to interactive marker server
    # instance = number of type instance
    # frame = frame for hand in rviz
    # hand => use HandIdentifier to determine which hand to create
    # color => color of hands
    # config_file = file that provides mesh path, offset and ready pose for robot hand
    # (could be Valkyrie, R2, etc)
    def __init__(self, server, instance, hand, color, hand_state = OPEN):
        MarkerTemplate.__init__(self, server, instance, hand)
        self._id = instance
        self._color = color
        self._handState = hand_state
        if hand is LEFT:
            self._left = True
            self._right = False
        else:
            self._left = False
            self._right = True

        if not self.setup():
            self = None

    @property
    def setFrameID(self, frame):
        self._frameID = frame

    def getFrameID(self):
        return self._frameID

    def setup(self):
        self._tfListener = tf.TransformListener()
        self._syncRotation = Pose()
        self._center = Pose()
        self._leftRobotActual = Pose()
        self._rightRobotActual = Pose()

        self._groupNames = {}
        self._groupPoseData = {}
        self._controlFrames = {}
        self._storedPoses = {}
        self._groupMenuHandlers = {}
        self._poseUpdateThreads = {}
        self._posesStored = {}
        self._endEffLinkData = {}

        self._nudgeDistance = 0.01
        robotNameList = rospy.get_param("robot_name")
        self._robotName = robotNameList[0]
        if(self.initMoveIt()):
            self.initMenu()
            self.initROS()
            self.initMarkers()
            self.initHand()

            #self._hand = self.addHand(self._meshOffset, self._meshPath)

            # for printing hand positions
            self._handPositionIndex = 0
            self.menu_handler.reApply(self.server)
            self.server.applyChanges()
            return True
        else:
            return False


    def initMoveIt(self):
        self._moveitIF = MoveItInterface(self._robotName,self._robotName + "_moveit_config")
        self._robotFrameID = self._moveitIF.get_planning_frame()
        # add user specified goups
        if self._left:
            self._groupNames[MANIPULATOR] = "left_arm"
        else:
            self._groupNames[MANIPULATOR] = "right_arm"
        print "HandMarker.initMoveIt: manipulator group name is " + self._groupNames[MANIPULATOR]
        if (self._moveitIF.add_group(self._groupNames[MANIPULATOR], group_type=MANIPULATOR)):
            # append group list with auto-found end effectors
            for n in self._moveitIF.get_end_effector_names():
                rospy.loginfo("HandMarker.initMoveIt: end effector name is " + n)
                if "hand" in name:
                    if self._left:
                        if "left" in n:
                            self._groupNames[ENDEFFECTOR] = n
                    else:
                        if "right" in n:
                            self._groupNames[ENDEFFECTOR] = n
            # set the control frames for all types of groups
            for t, g in self._groupNames:
                self._controlFrames[g] = self._moveitIF.get_control_frame(g)

            # what do we have?
            self._moveitIF.print_basic_info()

            # get stored poses from model
            for t, group in self._groupNames :
                self._storedPoses[group] = {}
                self._moveitIF.set_display_mode(group, "last_point")
                for state_name in self._moveitIF.get_stored_state_list(group) :
                    self._storedPoses[group][state_name] = self._moveitIF.get_stored_group_state(group, state_name)

            # set up end effector link data
            n = self._groupNames[ENDEFFECTOR]
            self.endEffLinkData = EndEffectorLinkData(self._moveitIF.get_control_frame(n), self.tfListener)
            self.endEffLinkData[n].populate_data(self._moveitIF.get_group_links(n), self._moveitIF.get_urdf_model())

            return True
        else:
            rospy.logerr("Could not initialize MoveIt interface")
            return False

    def initMenu(self):

        self._menuList = []
        self._menuList.append(("Stored Poses", False))
        self._menuList.append(("Sync To Actual", False))
        self._menuList.append(("Show Path", True))
        self._menuList.append(("Execute", False))

        self.menu_handler.insert("Sync with robot hand", callback=self.syncActualPoses)
        self.menu_handler.insert("Plan", callback=self.planHand)
        self.menu_handler.insert("Execute", callback=self.executeHand)
        self.menu_handler.insert("Go to Ready Pose", callback=self.publishReadyPose)
        self.menu_handler.setCheckState(self.menu_handler.insert("Show hand", callback=self.enableShowHands), MenuHandler.CHECKED)
        self.menu_handler.setCheckState(self.menu_handler.insert("Show hand controls", callback=self.enableShowHandControls), MenuHandler.CHECKED)
        self.initNudgeMenu()

    def initROS(self):
        topic = "/" + self._hand + "_hand_goal/marker"
        self._handPub = rospy.Publisher(topic, Marker)
        rospy.Subscriber(str(self._robotName + "/joint_states"), JointState, self.jointStateCallback)

    def initMarkers(self) :
        print "HandMarker.initMarkers"
        # create 6DOF marker for manipulator
        name = self._groupNames[MANIPULATOR] + self._id
        self._intMarker = CreateInteractiveMarker(self._frameID, name, 0.2)
        self._intMarker.controls.extend(Create6DOFControls())
        self.addInteractiveMarker(self._intMarker, self.processHandFeedback)
        self.attachMenuHandler(self._intMarker)

        # what does this do?
        # self.resetGroupMarker(group)

        # # Set up stored pose sub menu
        # self.setupStoredPoseMenu(group)

        # add menus to server
        self._markerMenus[group].apply( self.server, group )
        self.server.applyChanges()


    def initHand(self):
        print "HandMarker.initHand"
        # what form is hand?
        # get stored pose that matches hand form
        # get stored poses from model
        for group in self._groupNames :
            print "HandMarker.initHand: group is " + group
            self.stored_poses[group] = {}
            self.moveit_interface.set_display_mode(group, "last_point")
            for state_name in self.moveit_interface.get_stored_state_list(group) :
                self.stored_poses[group][state_name] = self.moveit_interface.get_stored_group_state(group, state_name)


        print "HandMarker.initHand: about to do end effector stuff"
        #
        # create visual meshes for end effector bits (fingers, palm)
        group = self._groupNames[ENDEFFECTOR]
        for link in self._endEffLinkData[group].get_links() :
            if self._endEffLinkData[group].get_link_data(link) :
                if link != self._controlFrames[group]:
                    # get pose of link in parent frame from
                    (mesh, pose) = self._endEffLinkData[group].get_link_data(link)
                    marker = CreateMeshMarker(pose, mesh, alpha=0.1, sf=1.02)
                    marker.text = link
                    menu_control.markers.append( marker )

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
            # get mesh from move it interface
            self._meshPath = "package://marker_templates/resources/models/Hands/LeftHand.stl"
            # rotation for the visual model to align with robot
            q = kdl.Rotation.RPY(-1.57, 0, -1.57).GetQuaternion()
        else:
            self._meshPath = "package://marker_templates/resources/models/Hands/RightHand.stl"
            # rotation for the visual model to align with robot
            q = kdl.Rotation.RPY(1.57, 0, -1.57).GetQuaternion()
        #add hand
        self._meshOffset = Quaternion()
        self._meshOffset.x = q[0]
        self._meshOffset.y = q[1]
        self._meshOffset.z = q[2]
        self._meshOffset.w = q[3]

    def addHand(self, offset_quat, meshPath):
        name = self._name + self._id
        # create IM in 'world' frame with name and scale factor of 0.2
        marker = CreateInteractiveMarker(self._frameID, name, 0.2)
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

    def executeHand(self, feedback):
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

    def processFeedback(self, feedback) :

        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            if feedback.marker_name in self._manipulatorGroupNames :
                self._posesStore[feedback.marker_name] = feedback.pose

        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            if feedback.marker_name in self._manipulatorGroupNames :
                pt = PoseStamped()
                pt.header = feedback.header
                pt.pose = feedback.pose
                rospy.loginfo("CREATING PLAN")
                self._moveitIF.groups[feedback.marker_name].clear_pose_targets()
                self._moveitIF.create_plan_to_target(feedback.marker_name, pt)

        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.marker_name in self.group_names :
                handle = feedback.menu_entry_id
                if handle == self._groupMenuHandlers[(feedback.marker_name,"Sync To Actual")] :
                    self.resetGroupMarker(feedback.marker_name)
                if handle == self._groupMenuHandlers[(feedback.marker_name,"Show Path")] :
                    state = self._markerMenus[feedback.marker_name].getCheckState( handle )
                    if state == MenuHandler.CHECKED:
                        self._markerMenus[feedback.marker_name].setCheckState( handle, MenuHandler.UNCHECKED )
                        self._moveitIF.set_display_mode(feedback.marker_name, "last_point")
                    else :
                        self._markerMenus[feedback.marker_name].setCheckState( handle, MenuHandler.CHECKED )
                        self._moveitIF.set_display_mode(feedback.marker_name, "all_points")
                if handle == self._groupMenuHandlers[(feedback.marker_name,"Execute")] :
                    r = self._moveitIF.execute_plan(feedback.marker_name)
                    if not r :
                        rospy.logerr(str("RobotTeleop::processFeedback(mouse) -- failed moveit execution for group: " + feedback.marker_name + ". re-synching..."))

        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE :
            if feedback.marker_name in self._manipulatorGroupNames :
                if not feedback.marker_name in self._posesStore: return
                p = toMsg(fromMsg(self._posesStore[feedback.marker_name]).Inverse()*fromMsg(feedback.pose))
                r = (kdl.Rotation.Quaternion(p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w)).GetRPY()
                # print "delta p: ", p
                axis_name =  feedback.control_name
                axis_id = self.axisMap(axis_name)
                axis_delta = self.getAxis(axis_name, p, r)

                self._moveitIF.groups[feedback.marker_name].shift_pose_target(axis_id, axis_delta)
                self._posesStore[feedback.marker_name] = feedback.pose
                # print self._moveitIF.groups[feedback.marker_name].plan()

        self._markerMenus[feedback.marker_name].reApply( self.server )
        self.server.applyChanges()

    # def processHandFeedback(self, feedback):
    #     self._hand.pose = feedback.pose

    def setupStoredPoseMenu(self, group) :
        for m,c in self.menu_options :
            if m == "Stored Poses" :
                sub_menu_handle = self._markerMenus[group].insert(m)
                for p in self._moveitIF.get_stored_state_list(group) :
                    self._groupMenuHandlers[(group,m,p)] = self._markerMenus[group].insert(p,parent=sub_menu_handle,callback=self.storedPoseCallback)
            else :
                self._groupMenuHandlers[(group,m)] = self._markerMenus[group].insert( m, callback=self.processFeedback )
                if c : self._markerMenus[group].setCheckState( self._groupMenuHandlers[(group,m)], MenuHandler.UNCHECKED )

    def startPoseUpdateThread(self, group) :
        self._groupPoseData[group] = PoseStamped()
        try :
            self._poseUpdateThreads[group] = PoseUpdateThread(group, self._frameID, self._controlFrames[group], self._tfListener, None)
            self._poseUpdateThreads[group].start()
        except :
            rospy.logerr("RobotTeleop::startPoseUpdateThread() -- unable to start group update thread")

    def jointStateCallback(self, data) :
        self._jointData = data

    def storedPoseCallback(self, feedback) :
        print "pose callback: "
        for p in self._moveitIF.get_stored_state_list(feedback.marker_name) :
            if self.group_menu_handles[(feedback.marker_name,"Stored Poses",p)] == feedback.menu_entry_id :
                self._moveitIF.create_joint_plan_to_target(feedback.marker_name, self.stored_poses[feedback.marker_name][p])
                r = self._moveitIF.execute_plan(feedback.marker_name)
                if not r : rospy.logerr(str("RobotTeleop::process_feedback(pose) -- failed moveit execution for group: " + feedback.marker_name + ". re-synching..."))
                if self._moveitIF.get_group_type(feedback.marker_name) == "manipulator" :
                    rospy.sleep(3)
                    self.resetGroupMarker(feedback.marker_name)

    def resetGroupMarker(self, group) :
        _marker_valid = False
        while not _marker_valid:
            if self.pose_update_thread[group].is_valid:
                self.group_pose_data[group] = copy.deepcopy(self.pose_update_thread[group].get_pose_data())
                self.server.setPose(self.markers[group].name, self.group_pose_data[group])
                self.server.applyChanges()
                self.server.setPose(self.markers[group].name, self.group_pose_data[group])
                self.server.applyChanges()
                _marker_valid = True
            rospy.sleep(0.1)

    def axisMap(self, n) :
        if n == "move_x": return 0
        elif n == "move_z": return 1
        elif n == "move_y": return 2
        elif n == "rotate_x": return 3
        elif n == "rotate_z": return 4
        elif n == "rotate_y": return 5

    def getAxis(self, n, p, r) :
        d = 0
        if n == "move_x": d=p.position.x
        elif n == "move_z": d=p.position.y
        elif n == "move_y": d=p.position.z
        elif n == "rotate_x": d=r[0]
        elif n == "rotate_z": d=r[1]
        elif n == "rotate_y": d=r[2]
        return d

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
