from nasa_robot_teleop.xml_reflection.basics import *
import nasa_robot_teleop.xml_reflection.core as xmlr

xmlr.start_namespace('atdf')


class DisplayObject(xmlr.Object):
    def __init__(self, name = None):
        self.aggregate_init()
        self.name = name

xmlr.reflect(DisplayObject, params = [
    xmlr.Attribute('name', str, False)
    ])


# class DisplayObjects(xmlr.Object):
#     def __init__(self, name = None):
#         self.aggregate_init()
#         self.objects = []

# xmlr.reflect(DisplayObjects, params = [
#     xmlr.Element('display_object', DisplayObject)
#     ])

# class EndEffectorWaypoint(xmlr.Object):
#     def __init__(self, id = None):
#         self.aggregate_init()
#         self.id = id

# xmlr.reflect(EndEffectorWaypoint, params = [
#     xmlr.Attribute('id', str, False)
#     ])

# class EndEffectorWaypoints(xmlr.Object):
#     def __init__(self):
#         self.aggregate_init()
#         self.waypoints = []

# xmlr.reflect(EndEffectorWaypoints, params = [
#     xmlr.Attribute('end_effector_waypoint', EndEffectorWaypoint)
#     ])

class AffordanceTemplate(xmlr.Object):
    def __init__(self, name = None):
        self.aggregate_init()

        self.name = name
        self.objects = []
        self.waypoints = []

        self.object_map = {}
        self.waypoint_map = {}

        # self.parent_map = {}
        # self.child_map = {}

    def add_aggregate(self, typeName, elem):
        xmlr.Object.add_aggregate(self, typeName, elem)

        if typeName == 'object':
            obj = elem
            self.object_map[obj.name] = obj
            # self.parent_map[ joint.child ] = (joint.name, joint.parent)
            # if joint.parent in self.child_map:
            #     self.child_map[joint.parent].append( (joint.name, joint.child) )
            # else:
            #     self.child_map[joint.parent] = [ (joint.name, joint.child) ]
        elif typeName == 'waypoint':
            waypoint = elem
            self.waypoint_map[waypoint.name] = waypoint

    def add_object(self, obj):
        self.add_aggregate('object', obj)

    def add_waypoint(self, waypoint):
        self.add_aggregate('waypoint', waypoint)

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
    xmlr.AggregateElement('display_object', DisplayObject),
    # xmlr.Attribute('end_effector_waypoints', EndEffectorWaypoints)
    ])

# Make an alias
RTDF = AffordanceTemplate

xmlr.end_namespace()


