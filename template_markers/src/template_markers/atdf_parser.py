from nasa_robot_teleop.xml_reflection.basics import *
import nasa_robot_teleop.xml_reflection.core as xmlr

xmlr.start_namespace('atdf')

verbose = True

class Pose(xmlr.Object):
    def __init__(self, xyz=None, rpy=None):
        self.xyz = xyz
        self.rpy = rpy

    def check_valid(self):
        assert self.xyz is not None or self.rpy is not None

    # Aliases for backwards compatibility
    @property
    def rotation(self): return self.rpy
    @rotation.setter
    def rotation(self, value): self.rpy = value
    @property
    def position(self): return self.xyz
    @position.setter
    def position(self, value): self.xyz = value

xmlr.reflect(Pose, params = [
    xmlr.Attribute('xyz', 'vector3', False),
    xmlr.Attribute('rpy', 'vector3', False)
    ])


class Controls(xmlr.Object):
    def __init__(self, xyz=None, rpy=None, scale=None):
        self.xyz = xyz
        self.rpy = rpy
        self.scale = scale

    def check_valid(self):
        assert self.xyz is not None or self.rpy is not None

xmlr.reflect(Controls, params = [
    xmlr.Attribute('xyz', 'vector3', False),
    xmlr.Attribute('rpy', 'vector3', False),
    xmlr.Attribute('scale', float),
    ])


# Common stuff
name_attribute = xmlr.Attribute('name', str)
origin_element = xmlr.Element('origin', Pose, False)
controls_element = xmlr.Element('controls', Controls, False)

class Color(xmlr.Object):
    def __init__(self, *args):
        # What about named colors?
        count = len(args)
        if count == 4 or count == 3:
            self.rgba = args
        elif count == 1:
            self.rgba = args[0]
        elif count == 0:
            self.rgba = None
        if self.rgba is not None:
            if len(self.rgba) == 3:
                self.rgba += [1.]
            if len(self.rgba) != 4:
                raise Exception('Invalid color argument count')

xmlr.reflect(Color, params = [
    xmlr.Attribute('rgba', 'vector4')
    ])


class Box(xmlr.Object):
    def __init__(self, size = None):
        self.size = size

xmlr.reflect(Box, params = [
    xmlr.Attribute('size', 'vector3')
    ])


class Cylinder(xmlr.Object):
    def __init__(self, radius = 0.0, length = 0.0):
        self.radius = radius
        self.length = length

xmlr.reflect(Cylinder, params = [
    xmlr.Attribute('radius', float),
    xmlr.Attribute('length', float)
    ])


class Sphere(xmlr.Object):
    def __init__(self, radius=0.0):
        self.radius = radius

xmlr.reflect(Sphere, params = [
    xmlr.Attribute('radius', float)
    ])


class Mesh(xmlr.Object):
    def __init__(self, filename = None, scale = None):
        self.filename = filename
        self.scale = scale

xmlr.reflect(Mesh, params = [
    xmlr.Attribute('filename', str),
    xmlr.Attribute('scale', 'vector3', required=False)
    ])

class Texture(xmlr.Object):
    def __init__(self, filename = None):
        self.filename = filename

xmlr.reflect(Texture, params = [
    xmlr.Attribute('filename', str)
    ])


class Material(xmlr.Object):
    def __init__(self, name=None, color=None, texture=None):
        self.name = name
        self.color = color
        self.texture = texture

    def check_valid(self):
        if self.color is None and self.texture is None:
            # xmlr.on_error("Material has neither a color nor texture\n")
            pass

xmlr.reflect(Material, params = [
    name_attribute,
    xmlr.Element('color', Color, False),
    xmlr.Element('texture', Texture, False)
    ])


class GeometricType(xmlr.ValueType):
    def __init__(self):
        self.factory = xmlr.FactoryType('geometric', {
            'box': Box,
            'cylinder': Cylinder,
            'sphere': Sphere,
            'mesh': Mesh
            })

    def from_xml(self, node):
        children = xml_children(node)
        assert len(children) == 1, 'One element only for geometric'
        return self.factory.from_xml(children[0])

    def write_xml(self, node, obj):
        name = self.factory.get_name(obj)
        child = node_add(node, name)
        obj.write_xml(child)

xmlr.add_type('geometric', GeometricType())


class DisplayObject(xmlr.Object):
    def __init__(self, name = None, geometry = None, origin = None, material = None, controls = None, parent = None):
        self.name = name
        self.origin = origin
        self.controls = controls
        self.geometry = geometry
        self.material = material
        self.parent = parent

xmlr.reflect(DisplayObject, params = [
    origin_element,
    controls_element,
    xmlr.Element('geometry', 'geometric'),
    xmlr.Element('material', Material, False),
    xmlr.Attribute('name', str, True),
    xmlr.Attribute('parent', str, False)
    ])

class DisplayObjects(xmlr.Object):
    def __init__(self, name = None):
        self.aggregate_init()
        self.display_objects = []
        self.display_object_map = {}

    def add_aggregate(self, typeName, elem):
        xmlr.Object.add_aggregate(self, typeName, elem)

        if typeName == 'display_object':
            display_object = elem
            self.display_object_map[display_object.name] = display_object

    def add_display_object(self, display_object):
        self.add_aggregate('display_object', display_object)

xmlr.reflect(DisplayObjects, params = [
    xmlr.AggregateElement('display_object', DisplayObject)
    ])




class EndEffectorWaypoint(xmlr.Object):
    def __init__(self, end_effector = None, id = None, display_object = None, origin = None, controls = None):
        self.id = id
        self.end_effector = end_effector
        self.display_object = display_object
        self.controls = controls
        self.origin = origin

xmlr.reflect(EndEffectorWaypoint, params = [
    origin_element,
    controls_element,
    xmlr.Attribute('id', str, True),
    xmlr.Attribute('end_effector', str, True),
    xmlr.Attribute('display_object', str, True)
    ])

class EndEffectorWaypoints(xmlr.Object):
    def __init__(self):
        self.aggregate_init()
        self.end_effector_waypoints = []
        self.end_effector_waypoint_map = {}

    def add_aggregate(self, typeName, elem):
        xmlr.Object.add_aggregate(self, typeName, elem)

        if typeName == 'end_effector_waypoint':
            end_effector_waypoint = elem
            self.end_effector_waypoint_map[end_effector_waypoint.id] = end_effector_waypoint

    def add_end_effector_waypoint(self, end_effector_waypoint):
        self.add_aggregate('end_effector_waypoint', waypoint)


xmlr.reflect(EndEffectorWaypoints, params = [
    xmlr.AggregateElement('end_effector_waypoint', EndEffectorWaypoint)
    ])

class AffordanceTemplateStructure(xmlr.Object):
    def __init__(self, name = None):
        self.aggregate_init()
        self.name = name
        self.display_objects = None
        self.end_effector_waypoints = None

    @classmethod
    def from_file(cls, key = 'template_description'):
        # Could move this into xml_reflection
        try:
            f = open(key)
        except IOError :
            f.close()
        return cls.from_xml_string(f.read())

xmlr.reflect(AffordanceTemplateStructure, tag = 'affordance_template', params = [
    xmlr.Attribute('name', str, True),
    xmlr.Attribute('image', str, False),
    xmlr.Element('display_objects', DisplayObjects),
    xmlr.Element('end_effector_waypoints', EndEffectorWaypoints)
    ])

# Make an alias
ATDF = AffordanceTemplateStructure

xmlr.end_namespace()


