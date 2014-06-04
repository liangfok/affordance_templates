from nasa_robot_teleop.xml_reflection.basics import *
import nasa_robot_teleop.xml_reflection.core as xmlr

xmlr.start_namespace('atdf')

xmlr.add_type('element_link', xmlr.SimpleElementType('link', str))
xmlr.add_type('element_xyz', xmlr.SimpleElementType('xyz', 'vector3'))

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


# Common stuff
name_attribute = xmlr.Attribute('name', str)
origin_element = xmlr.Element('origin', Pose, False)

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
    def __init__(self, name = None, geometry = None, origin = None, material = None):
        self.name = name
        self.origin = origin
        self.geometry = geometry
        self.material = material

xmlr.reflect(DisplayObject, params = [
    origin_element,
    xmlr.Element('geometry', 'geometric'),
    xmlr.Element('material', Material, False),
    xmlr.Attribute('name', str, True)
    ])

class DisplayObjects(xmlr.Object):
    def __init__(self, name = None):
        self.objects = []

xmlr.reflect(DisplayObjects, params = [
    xmlr.Element('display_object', DisplayObject)
    ])

# class EndEffectorWaypoint(xmlr.Object):
#     def __init__(self, id = None):
#         self.id = id

# xmlr.reflect(EndEffectorWaypoint, params = [
#     xmlr.Attribute('id', str, False)
#     ])

# class EndEffectorWaypoints(xmlr.Object):
#     def __init__(self):
#         self.waypoints = []

# xmlr.reflect(EndEffectorWaypoints, params = [
#     xmlr.Attribute('end_effector_waypoint', EndEffectorWaypoint)
#     ])

class AffordanceTemplate(xmlr.Object):
    def __init__(self, name = None):
        self.aggregate_init()

        self.name = name
        self.display_objects = []
        self.end_effector_waypoints = []

        self.display_object_map = {}
        self.end_effector_waypoint_map = {}

    def add_aggregate(self, typeName, elem):
        xmlr.Object.add_aggregate(self, typeName, elem)

        if typeName == 'display_object':
            display_object = elem
            self.display_object_map[display_object.name] = display_object
        elif typeName == 'end_effector_waypoint':
            end_effector_waypoint = elem
            self.end_effector_waypoint_map[end_effector_waypoint.name] = end_effector_waypoint

    def add_display_object(self, display_object):
        self.add_aggregate('display_object', display_object)

    def add_end_effector_waypoint(self, end_effector_waypoint):
        self.add_aggregate('end_effector_waypoint', waypoint)

    @classmethod
    def from_file(cls, key = 'template_description'):
        # Could move this into xml_reflection
        try:
            f = open(key)
        except IOError :
            f.close()
        return cls.from_xml_string(f.read())

xmlr.reflect(AffordanceTemplate, tag = 'template', params = [
#   name_attribute,
    xmlr.Attribute('name', str, True),
    xmlr.Attribute('image', str, False),
    xmlr.AggregateElement('display_object', DisplayObject)
    # xmlr.Attribute('end_effector_waypoints', EndEffectorWaypoints)
    ])

# Make an alias
RTDF = AffordanceTemplate

xmlr.end_namespace()


