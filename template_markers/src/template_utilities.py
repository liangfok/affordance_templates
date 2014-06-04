import copy
import rospy
import math

from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from nav_msgs.msg import Path
from interactive_markers import interactive_marker_server, menu_handler
from StringIO import StringIO
from random import randint
import PyKDL as kdl
import tf
import threading


def _CreateMarkerControl(name, orientation, marker_type):
    control = InteractiveMarkerControl()
    control.name = name
    control.orientation = orientation
    control.interaction_mode = marker_type
    control.always_visible = False
    return control

def _CreateTranslateXControl(name):
    orientation = Quaternion()
    orientation.x = 1
    orientation.w = 1
    return _CreateMarkerControl(name, orientation, InteractiveMarkerControl.MOVE_AXIS)

def _CreateTranslateYControl(name):
    orientation = Quaternion()
    orientation.z = 1
    orientation.w = 1
    return _CreateMarkerControl(name, orientation, InteractiveMarkerControl.MOVE_AXIS)

def _CreateTranslateZControl(name):
    orientation = Quaternion()
    orientation.y = 1
    orientation.w = 1
    return _CreateMarkerControl(name, orientation, InteractiveMarkerControl.MOVE_AXIS)

def _CreateRotateXControl(name):
    orientation = Quaternion()
    orientation.x = 1
    orientation.w = 1
    return _CreateMarkerControl(name, orientation, InteractiveMarkerControl.ROTATE_AXIS)

def _CreateRotateYControl(name):
    orientation = Quaternion()
    orientation.z = 1
    orientation.w = 1
    return _CreateMarkerControl(name, orientation, InteractiveMarkerControl.ROTATE_AXIS)

def _CreateRotateZControl(name):
    orientation = Quaternion()
    orientation.y = 1
    orientation.w = 1
    return _CreateMarkerControl(name, orientation, InteractiveMarkerControl.ROTATE_AXIS)

def _CreateTranslateXYControl(name):
    orientation = Quaternion()
    orientation.y = 1
    orientation.w = 1
    return _CreateMarkerControl(name, orientation, InteractiveMarkerControl.MOVE_PLANE)

def _CreateTranslateYZControl(name):
    orientation = Quaternion()
    orientation.z = 1
    orientation.w = 1
    return _CreateMarkerControl(name, orientation, InteractiveMarkerControl.MOVE_PLANE)

def _CreateTranslateXZControl(name):
    orientation = Quaternion()
    orientation.x = 1
    orientation.w = 1
    return _CreateMarkerControl(name, orientation, InteractiveMarkerControl.MOVE_PLANE)

def CreateTransRotControl(name):
    if name == 'TranslateX':
        return _CreateTranslateXControl(name)
    if name == 'TranslateY':
        return _CreateTranslateYControl(name)
    if name == 'TranslateZ':
        return _CreateTranslateZControl(name)
    if name == 'RotateX':
        return _CreateRotateXControl(name)
    if name == 'RotateY':
        return _CreateRotateYControl(name)
    if name == 'RotateZ':
        return _CreateRotateZControl(name)
    if name == 'TranslateXY':
        return _CreateTranslateXYControl(name)
    if name == 'TranslateYZ':
        return _CreateTranslateYZControl(name)
    if name == 'TranslateXZ':
        return _CreateTranslateXZControl(name)
    if name == 'TranslateXZ': 
      return _CreateTranslateXZControl(name)

def CreateInteractiveMarker(frame_id, name, scale):
    interactive_marker = InteractiveMarker()
    interactive_marker.header.frame_id = frame_id
    interactive_marker.name = name
    interactive_marker.description = name
    interactive_marker.scale = scale
    return interactive_marker

def CreatePrimitiveMarker(scaleFactor, marker_type, id=randint(0,10000)):
    marker = Marker()
    marker.ns = "visual"
    marker.id = id
    marker.scale.x = scaleFactor
    marker.scale.y = scaleFactor
    marker.scale.z = scaleFactor
    marker.type = marker_type
    marker.pose.orientation.w = 1.0
    return marker

def CreateMeshMarker(pose, mesh_path, alpha = 1, scaleFactor=1, id=randint(0,10000)):
    marker = Marker()
    marker.ns = "visual"
    marker.id = id
    marker.scale.x = scaleFactor
    marker.scale.y = scaleFactor
    marker.scale.z = scaleFactor
    marker.pose = pose
    marker.type = Marker.MESH_RESOURCE
    marker.mesh_use_embedded_materials = True
    marker.mesh_resource = mesh_path
    marker.frame_locked = True
    return marker

def CreatePrimitiveControl(name, scaleFactor, marker_type, id=randint(0,10000)):
    marker = CreatePrimitiveMarker(name, scaleFactor, marker_type, id)
    control = InteractiveMarkerControl()
    control.name = name
    control.always_visible = True
    control.interaction_mode = InteractiveMarkerControl.MENU
    control.markers.append(marker)
    return control

def CreateVisualControlFromMarker(marker, always_visible=True, interaction_mode=InteractiveMarkerControl.MENU):
    control = InteractiveMarkerControl()
    control.name = "visual"
    control.always_visible = always_visible
    control.interaction_mode = interaction_mode
    control.markers.append(marker)
    return control

def Create6DOFControls(prefix=None):
    """Creates 6 DOF controls.

    @type prefix: string
    @param prefix: A prefix used when naming the InteractiveMarkerControls.

    @rtype: list of InteractiveMarkerControls
    @return: A list containing 6 InteractiveMarkerControls for translation/rotation.
    """
    controls = []
    controls.extend(CreateTranslateControls(prefix))
    controls.extend(CreateRotateControls(prefix))
    return controls

def CreateTranslateControls(prefix=None):
    """Creates three InteractiveMarkerControl objects for translation.

    @type prefix: string
    @param prefix: A prefix used when naming the InteractiveMarkerControls.

    @rtype: list of InteractiveMarkerControls
    @return: The three InteractiveMarkerControls for translation.
    """
    controls = []
    if prefix:
        controls.append(CreateTransRotControl(prefix + "TranslateX"))
        controls.append(CreateTransRotControl(prefix + "TranslateY"))
        controls.append(CreateTransRotControl(prefix + "TranslateZ"))
    else:
        controls.append(CreateTransRotControl("TranslateX"))
        controls.append(CreateTransRotControl("TranslateY"))
        controls.append(CreateTransRotControl("TranslateZ"))
    return controls

def CreateRotateControls(prefix=None):
    """Creates three InteractiveMarkerControl objects for roll, pitch, yaw.

    @type prefix: string
    @param prefix: A prefix used when naming the InteractiveMarkerControls.

    @rtype: list of InteractiveMarkerControls
    @return: The three InteractiveMarkerControls for rotation.
    """
    controls = []
    if prefix:
        controls.append(CreateTransRotControl(prefix + "RotateX"))
        controls.append(CreateTransRotControl(prefix + "RotateY"))
        controls.append(CreateTransRotControl(prefix + "RotateZ"))
    else:
        controls.append(CreateTransRotControl("RotateX"))
        controls.append(CreateTransRotControl("RotateY"))
        controls.append(CreateTransRotControl("RotateZ"))
    return controls

def ScaleMarker(marker_template, control_scale=None, visual_scale=None):
    """Scale InteractiveMarker and/or a visual Marker associated with the InteractiveMarker.

    @type marker_template: subclass of MarkerTemplate()
    @param marker_template: The template object containing InteractiveMarkers.

    @type control_scale: float
    @param control_scale: The scale factor for the InteractiveMarker.

    @type visual_scale: geometry_msgs/Vector3
    @param visual_scale: The scale factor for the visualization Marker in the template.
    """
    server = marker_template.server
    menu_handler = marker_template.menu_handler
    marker_name = marker_template.key
    if server:
        current_marker = server.get(marker_name)
        if current_marker:
            
            # rescale marker
            marker = Marker()
            marker = GetVisualMarker(current_marker)
            if visual_scale is not None:
                marker.scale = visual_scale

            # push marker into visual control
            visual = InteractiveMarkerControl()
            visual.name = "visual"
            visual.always_visible = GetVisualControl(current_marker).always_visible
            visual.interaction_mode = GetVisualControl(current_marker).interaction_mode
            visual.orientation = GetVisualControl(current_marker).orientation
            visual.markers.append(marker)

            new_marker = InteractiveMarker()
            new_marker.header.frame_id = current_marker.header.frame_id
            new_marker.name = current_marker.name
            new_marker.description = current_marker.description
            new_marker.pose = current_marker.pose
            new_marker.scale = current_marker.scale
            if control_scale is not None:
                new_marker.scale = control_scale

            new_marker.controls.append(visual)

            for control in current_marker.controls:
                if 'Translate' in control.name or 'Rotate' in control.name:
                    # todo rename Plane Translate so we don't need to do this extra check
                    if control.name not in ['TranslateXY', 'TranslateYZ','TranslateXZ']:
                        new_marker.controls.append(CreateTransRotControl(control.name))

            # insert the updated marker into the server
            server.insert(new_marker)
            menu_handler.apply(server, marker_name)

# normalizes a Quaternion()
def normalize(q):
    q = quaternion_to_nparray(q)
    sq = sum([x*x for x in q])
    inv = 1.0 / sq
    qnorm = [x*inv for x in q]
    quat = Quaternion()
    quat.x = qnorm[0]
    quat.y = qnorm[1]
    quat.z = qnorm[2]
    quat.w = qnorm[3]
    return quat

# convert a Quaternion() message to a numpy array
# q = [x y z w]
def quaternion_to_nparray(quaternion):
    array = []
    array.append(quaternion.x)
    array.append(quaternion.y)
    array.append(quaternion.z)
    array.append(quaternion.w)
    return array

def getPoseFromFrame(frame):
    """
    Return a Pose() message from a KDL frame.

    @type frame: PyKDL.Frame
    @param frame: The desired frame.

    @rtype: geometry_msgs/Pose
    @return: The Pose from the KDL frame.
    """
    pose = Pose()
    vector = frame.p
    pose.position.x = vector.x()
    pose.position.y = vector.y()
    pose.position.z = vector.z()

    q = frame.M.GetQuaternion()

    # normalize
    qmag = math.sqrt(q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2)
    qnorm = []
    qnorm.append(q[0] / qmag)
    qnorm.append(q[1] / qmag)
    qnorm.append(q[2] / qmag)
    qnorm.append(q[3] / qmag)

    pose.orientation.x = qnorm[0]
    pose.orientation.y = qnorm[1]
    pose.orientation.z = qnorm[2]
    pose.orientation.w = qnorm[3]

    return pose

def getFrameFromPose(pose):
    """
    Return a PyKDL.Frame object from a Pose().

    @type pose: geometry_msgs/Pose
    @param pose: Pose.

    @rtype: PyKDL.Frame
    """
    V = kdl.Vector(pose.position.x, pose.position.y, pose.position.z)
    x = pose.orientation.x
    y = pose.orientation.y
    z = pose.orientation.z
    w = pose.orientation.w
    qmag = math.sqrt(x**2 + y**2 + z**2 + w**2)
    R = kdl.Rotation.Quaternion(pose.orientation.x / qmag, pose.orientation.y / qmag, pose.orientation.z / qmag, pose.orientation.w / qmag)
    return kdl.Frame(R, V)
    
def getPoseFromRobotFrame(tf_listener, robot_frame, offset):
    """Return a pose relative to the robot.

    @type tf_listener TransformListener
    @param tf_listener The registered TF listener.

    @type robot_frame string
    @param robot_frame Robot frame e.g. "v1/Pelvis"

    @type offset Pose
    @param offset Offset of the marker relative to the robot's current position.

    @rtype Pose
    @returns The relative position from the robot if TF is available, None otherwise.
    """
    ret_pose = getPoseStampedFromRobotFrame(tf_listener, robot_frame, offset)
    if ret_pose != None:
        return ret_pose.pose


def getPoseStampedFromRobotFrame(tf_listener, robot_frame, offset):
    """Return a pose relative to the robot.

    @type tf_listener TransformListener
    @param tf_listener The registered TF listener.

    @type robot_frame string
    @param robot_frame Robot frame e.g. "v1/Pelvis"

    @type offset Pose
    @param offset Offset of the marker relative to the robot's current position.

    @rtype PoseStamped
    @returns The relative position from the robot if TF is available, None otherwise.
    """
    robot_id = robot_frame
    world_id = 'world'
    start_pose = PoseStamped()
    start_pose.header.seq = 0
    start_pose.header.stamp = rospy.Time(0)
    start_pose.header.frame_id = robot_id
    start_pose.pose = offset
    count = 0
    ret_pose = None
    try:
        t = rospy.Time(0)
        tf_listener.waitForTransform(robot_id, world_id, t, rospy.Duration(3))
        ret_pose = tf_listener.transformPose(world_id, start_pose)
    except:
        pass

    return ret_pose

def writePoseArrayToFile(filename, posearray):
    
    try:
        fileStr = StringIO()
        posearray.serialize(fileStr)
        # write file
        with open(filename, "w") as f:
            f.write(fileStr.getvalue())
    except:
        pass

def readPoseArrayFromFile(filename):
    try: 
        # read file
        with open(filename) as f:
            poseArrayString = f.read()
        # deserialize it into PoseArray
        pa = PoseArray()
        pa.deserialize(poseArrayString)
        return pa
    except:
        print "Returning no pose array for " + filename
        return None

def readPathFromFile(filename):
    try: 
        # read file
        with open(filename) as f:
            pathString = f.read()
        # deserialize it into PoseArray
        path = Path()
        print "about to deserialize"
        path.deserialize(pathString)
        print "deserialized"
        return path
    except:
        print "Returning no path for " + filename
        return None

def writePathToFile(filename, path):
    
    try:
        fileStr = StringIO()
        path.serialize(fileStr)
        # write file
        with open(filename, "w") as f:
            f.write(fileStr.getvalue())
    except:
        pass

def processPath(poseArray, pointsPerSegement):

    N = len(poseArray.poses)

    # 20 steps
    tt = np.linspace(0.0, 1.0, N*pointsPerSegement)
    points = []
    quaternions = []

    # get all cones
    for p in poseArray.poses:
        points.append(p.position)
        quaternions.append(p.orientation)

    pos = []
    ori = []

    # if 2 points (cones), just interpolate the path between them
    if N == 2:
        xvals = (points[1].x - points[0].x)*tt
        yvals = (points[1].y - points[0].y)*tt
        zvals = (points[1].z - points[0].z)*tt
        for i in range(len(xvals)):
            point = Point()
            point.x = points[0].x + xvals[i]
            point.y = points[0].y + yvals[i]
            point.z = points[0].z + zvals[i]
            pos.append(point)
        for t in tt:
            ori.append(nlerp(quaternions[0], quaternions[1], t))

    # if > 3, catmull-rom
    # http://en.wikipedia.org/wiki/Cubic_Hermite_spline#Catmull.E2.80.93Rom_spline
    elif N >= 3:
        n = N - 1
        # first spline, use the first point (cone) for P0, P1
        for t in tt:
            pos.append(catmull_rom(t, points[0], points[0], points[1], points[2]))
            ori.append(nlerp(quaternions[0], quaternions[1], t))

        x = 0
        while x < (N-3):
            for t in tt:
              pos.append(catmull_rom(t, points[x], points[x+1], points[x+2], points[x+3]))
              ori.append(nlerp(quaternions[x+1], quaternions[x+2], t))
            x +=1

        # last spline, use the last point (cone) for P2, P3
        for t in tt:
            pos.append(catmull_rom(t, points[n-2], points[n-1], points[n], points[n]))
            ori.append(nlerp(quaternions[n-1], quaternions[n], t))

    # publish the points as a Path() message
    path = Path()
    pArray = PoseArray()
    path.header.frame_id = poseArray.header.frame_id
    pArray.header.frame_id = poseArray.header.frame_id
    for i in range(len(pos)):
        poseStamped = PoseStamped()
        poseStamped.header.frame_id = poseArray.header.frame_id
        poseStamped.pose.position = pos[i]
        poseStamped.pose.orientation = ori[i]

        pose = Pose()
        pose.position = pos[i]
        pose.orientation = ori[i]

        pArray.poses.append(pose)
        path.poses.append(poseStamped)


    return (path, pArray)

class PoseUpdateThread(threading.Thread) :
    def __init__(self, name, root_frame, control_frame, tf_listener, offset_pose) :
        super(PoseUpdateThread,self).__init__()
        self.name = name
        self.pose_data = PoseStamped()
        self.tf_listener = tf_listener
        self.control_frame = control_frame
        self.root_frame = root_frame
        self.is_valid = False
        self.offset_pose = offset_pose
        if offset_pose != None :
            self.T_offset = getFrameFromPose(self.offset_pose)

    def run(self) :
        while True :
            try :
                self.tf_listener.waitForTransform(self.control_frame,self.root_frame, rospy.Time(0), rospy.Duration(2.0))
                (trans, rot) = self.tf_listener.lookupTransform(self.root_frame, self.control_frame, rospy.Time(0))
                p = Pose()
                p.position.x = trans[0]
                p.position.y = trans[1]
                p.position.z = trans[2]
                p.orientation.x = rot[0]
                p.orientation.y = rot[1]
                p.orientation.z = rot[2]
                p.orientation.w = rot[3]
                if self.offset_pose != None :
                    T = getFrameFromPose(P)
                    self.pose_data = toMsg(T*self.T_offset)
                else :
                    self.pose_data = P
                self.is_valid = True
            except :
                rospy.logdebug("PoseUpdateThread::run() -- could not update thread")
            rospy.sleep(0.1)

    def get_pose_data(self) :
        self.is_valid = False
        return self.pose_data

class EndEffectorLinkData :

    def __init__(self, root_frame, tf_listener) :

        self.link_meshes = {}
        self.link_origins = {}
        self.offset_pose_data = {}
        self.offset_update_thread = {}
        self.links = []
        self.root_frame = root_frame
        self.tf_listener = tf_listener

    def add_link(self, link, mesh, origin) :
        self.link_meshes[link] = mesh
        self.link_origins[link] = origin
        self.links.append(link)

    def populate_data(self, links, urdf) :

        for link in links :
            if not link in urdf.link_map :
                print "EndEffectorLinkData::populate_data() -- link: ", link, " not found in URDF model"
                return

            model_link = urdf.link_map[link]

            if model_link :
                if model_link.visual  :
                    if model_link.visual.geometry  :
                        if model_link.visual.geometry.filename  :
                            mesh = model_link.visual.geometry.filename

                            p = Pose()
                            rpy = RPY(model_link.visual.origin.rpy[0],model_link.visual.origin.rpy[1],model_link.visual.origin.rpy[2])
                            q = rpy.GetQuaternion()
                            p.position.x = model_link.visual.origin.xyz[0]
                            p.position.y = model_link.visual.origin.xyz[1]
                            p.position.z = model_link.visual.origin.xyz[2]
                            p.orientation.x = q[0]
                            p.orientation.y = q[1]
                            p.orientation.z = q[2]
                            p.orientation.w = q[3]
                            self.add_link(link, mesh, p)

        self.start_offset_update_thread()

    def has_link(self, link) :
        return link in self.links

    def get_links(self) :
        return self.links

    def get_link_data(self, link) :
        if not self.has_link(link): 
            return False
        if not self.offset_update_thread[link].get_pose_data(): 
            return False
        return (self.link_meshes[link], self.offset_update_thread[link].get_pose_data())

    def start_offset_update_thread(self) :
        for link in self.links :
            self.offset_pose_data[link] = PoseStamped()
            try :
                self.offset_update_thread[link] = PoseUpdateThread(link, self.root_frame, link, self.tf_listener, self.link_origins[link])
                self.offset_update_thread[link].start()
            except :
                rospy.logerr("EndEffectorLinkData::start_offset_update_thread() -- unable to start end effector link offset update thread")

    def get_link_offset(self, link) :
        return self.offset_pose_data[link]
