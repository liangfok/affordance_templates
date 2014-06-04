import os
import subprocess
import imp
import sys
import signal
import pprint

import rospy
import zmq
import template_markers
import template_server_protobuf

from AffordanceTemplateServerCmd_pb2 import Template, Request, Response, Pose, Position, Orientation
from visualization_msgs.msg import Marker

from threading import Thread

class AffordanceTemplateServer(Thread):
    """Template server."""
    def __init__(self, topic_arg=None):
        Thread.__init__(self)
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        self.topic_arg = topic_arg

        # get exported templates from affordance_templates package
        # self._plugin_description = self.getPluginDescription("template_markers")
        # self._resource_path = self.getPackagePath("template_markers")
        # self._resource_path = os.path.join(self._resource_path, 'resources', 'rviz')
        # self._template_path = self.getTemplatePath()
        # self.class_map = self.getAvailableTemplates(self.plugin_description)

        # get path to template marker package
        self._package_path = self.getPackagePath("template_markers")
        # get path to actual template source files
        self._template_path = os.path.join(self._package_path, 'src')
        # get path to resources
        self._resource_path = os.path.join(self._package_path, 'resources', 'rviz')
        # find plugin_description.xml file that holds all templates
        filename = self._package_path + "/plugin_description.xml"
        # call getAvailableTemplates to parse out templates
        self.class_map = self.getAvailableTemplates(filename)

    @property
    def resource_path(self):
        """Return the path to the RViz images."""
        return str(self._resource_path)

    @property
    def plugin_description(self):
        """Return the path to the plugin_description.xml associated with the server."""
        return str(self._plugin_description[0])

    @property
    def template_path(self):
        """Return the path to the template nodes."""
        return str(self._template_path)

    def configureServer(self):
        """Configure the server's ZMQ ports and ROS subscribers."""

        # TODO: add dynamic reconfigure to change subscriber topic
        # configure ROS subscriber for bootstrapping templates
        sub = rospy.Subscriber("/foo", Marker, self.markerSub)

        # init zmq to port 6789
        context = zmq.Context()
        self.socket = context.socket(zmq.REP)
        self.socket.bind("tcp://*:6789")
        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)
        print "Afford Template Server started on port 6789"

    def run(self):

        self.configureServer()
        # process mapping
        while not rospy.is_shutdown():

            # poll every second
            try:
                socks = self.poller.poll(1000)
            except zmq.ZMQError, e:
                break

            for sock, state in socks:

                if state != zmq.POLLIN:
                    continue

                msg = sock.recv()

                # parse string to protobuf message
                request = Request()
                request.ParseFromString(msg)

                # for template in request.marker_template:
                #     rospy.loginfo('received: ' + str(template.type))

                response = Response()
                response.success = False
                # respond with available templates
                if request.type == request.QUERY:
                    try:
                        for class_type in self.class_map.iterkeys():
                            template = response.marker_template.add()
                            template.type = class_type
                            template.image_path = os.path.join(self.resource_path, ''.join([class_type, '.jpg']))
                        response.success = True
                    except:
                        print 'Error with query for available templates'

                # add a new template and add it to the mapping
                elif request.type == request.ADD:
                    try:
                        for template in request.marker_template:
                            class_type = str(template.type)
                            new_id = self.getNextID(class_type)
                            popen = self.addTemplate(class_type, new_id)
                            if popen != None:
                                self.class_map[class_type][new_id] = popen
                        response.success = True
                    except:
                        print 'Error adding template to server'

                # respond with a list of the running templates on the server
                elif request.type == request.RUNNING:
                    try:
                        # push the running templates into a temporary list so we can sort them
                        running_templates = []
                        for template in self.class_map.iterkeys():
                            running_templates.append(template)
                        running_templates.sort()

                        for class_type in self.class_map.iterkeys():
                            for id_ in self.class_map[class_type].iterkeys():
                                template = response.marker_template.add()
                                template.type = class_type
                                template.id = id_

                        response.success = True
                    except:
                        print 'Error parsing running templates'

                elif request.type == request.KILL:
                    try:
                        for template in request.marker_template:
                            self.removeTemplate(template.type, template.id)
                    except:
                        print 'Error trying to kill template'

                self.socket.send(response.SerializeToString())

    def removeTemplate(self, class_type, instance_id):
        """Stop a template process and remove it from the server's map.

        @type class_type string
        @param class_type The class type e.g. "Wheel", "Car", etc.

        @type instance_id int
        @param instance_id The ID of this instance.

        @rtype bool
        @returns True if process was stopped/removed.
        """
        if class_type in self.class_map and instance_id in self.class_map[class_type]:
            self.class_map[class_type][instance_id].terminate()
            del self.class_map[class_type][instance_id]

    def addTemplate(self, class_type, instance_id):
        """Start a template process using subprocess.Popen.

        @type class_type string
        @param class_type The class type e.g. "Wheel", "Car", etc.

        @type instance_id int
        @param instance_id The ID of this instance.

        @rtype int
        @returns The Popen object started by the server.
        """
        if class_type in self.class_map:
            filename = os.path.join(self.template_path, ''.join([class_type, '.py']))
            if self.topic_arg is None:
                args = [filename, str(instance_id), "True"]
            else:
                args = [filename, str(instance_id)]

            print("templateServer.addTemplate: adding template " + str(class_type))
            return subprocess.Popen(args)


    def getNextID(self, class_type):
        ids = self.class_map[class_type].keys()
        i = 0
        while True:
            if i in ids:
                i += 1
            else:
                return i

    def getPackagePath(self, pkg):
        """Return the path to the ROS package."""
        import rospkg
        rp = rospkg.RosPack()
        return rp.get_path(pkg)

    def getTemplatePath(self):
        """Return the path to the template nodes."""
        import rospkg
        rp = rospkg.RosPack()
        return os.path.join(rp.get_path('template_markers'), 'src', 'template_markers')

    def getPluginDescription(self, pkg):
        """Return the plugin_description.xml for a ROS package."""
        import rospkg
        rp = rospkg.RosPack()
        man = rp.get_manifest(pkg)
        return man.get_export(pkg, 'plugin')

    def getAvailableTemplates(self, templates):
        """Parse affordance_templates manifest for available classes."""
        # if os.path.exists(manifest):
        from xml.etree.ElementTree import ElementTree
        class_map = {}
        # parse manifest.xml
        tree = ElementTree()
        tree.parse(templates)

        # get all <class> tags
        for c in tree.findall('class'):
            # starting with groovy, 'name' tag is optional
            if 'name' in c:
                class_map[self.getRawName(c.get('name'))] = {}
            else:
                class_map[self.getRawName(c.get('type'))] = {}

        return class_map
        # else:
        #     print manifest, 'does not exist!'
        #     return {}

    def getRawName(self, name):
        """Parse the class_name and return just the type."""
        if '/' in name:
            return name.rsplit('/',1)[1]
        if '::' in name:
            return name.rsplit('::',1)[1]

    def markerSub(self, data):
        print 'markerSub: ', data