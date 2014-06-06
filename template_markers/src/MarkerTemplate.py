#!usr/bin/env python
import rospy
from interactive_markers.menu_handler import MenuHandler
from template_utilities import *

class MarkerTemplate(object):
    def __init__(self, server, id_, name, initial_pose=None):
        self._menu_handler = MenuHandler()
        self._menu_handler.insert("Delete", callback=self.deleteCallback)
        self._server = server
        self._key = name + str(id_)
        self._marker_map = {}
        self._callback_map = {}
        # not all templates will have a dynamic reconfigure server
        self._dserver = None
        self._initial_pose = initial_pose

    @property
    def initial_pose(self):
        """Return a marker's initial pose if it was passed in."""
        return self._initial_pose

    # @property
    # def key(self):
    #     """Get the marker template's key on the server."""
    #     key = self._key + str(self._id)
    #     return key

    @property
    def server(self):
        """Get the marker template's interactive marker server."""
        return self._server

    @property
    def marker_map(self):
        """Get the interactive marker map of this marker template."""
        return self._marker_map

    @property
    def callback_map(self):
        """Get the callback map of this marker template."""
        return self._callback_map

    @property
    def menu_handler(self):
        """Get the menu handler of this marker template."""
        return self._menu_handler

    @property
    def setNudgeDistance(self, distance):
        self._nudgeDistance = distance

    def getNudgeDistance(self):
        return self._nudgeDistance

    def initNudgeMenu(self):
        nudgeControl = self.menu_handler.insert("Nudge hand " + str(self._nudgeDistance) + "m")
        self.menu_handler.insert("World X", parent=nudgeControl, command_type=MenuEntry.FEEDBACK, command="", callback=self.nudgeX)
        self.menu_handler.insert("World Y", parent=nudgeControl, command_type=MenuEntry.FEEDBACK, command="", callback=self.nudgeY)
        self.menu_handler.insert("World Z", parent=nudgeControl, command_type=MenuEntry.FEEDBACK, command="", callback=self.nudgeZ)
        self.menu_handler.insert("World -X", parent=nudgeControl, command_type=MenuEntry.FEEDBACK, command="", callback=self.nudgeXNeg)
        self.menu_handler.insert("World -Y", parent=nudgeControl, command_type=MenuEntry.FEEDBACK, command="", callback=self.nudgeYNeg)
        self.menu_handler.insert("World -Z", parent=nudgeControl, command_type=MenuEntry.FEEDBACK, command="", callback=self.nudgeZNeg)

    def addInteractiveMarker(self, marker, callback=None):
        name = marker.name
        self._marker_map[name] = marker
        if callback:
            self._callback_map[name] = callback
            return self.server.insert(marker, callback)
        else:
            self._callback_map[name] = self.processFeedback
            return self.server.insert(marker, self.processFeedback)

    def removeInteractiveMarker(self, marker_name):
        if self.server.erase(marker_name):
            if marker_name in self._marker_map:
                del self._marker_map[marker_name]
                del self._callback_map[marker_name]
                return True
        return False

    def attachMenuHandler(self, marker):
        return self.menu_handler.apply(self.server, marker.name)

    def getMarker(self):
        return self.server.get(self._key)

    def hasMarker(self):
        if self._key in self._marker_map.keys():
            return True
        else:
            return False

    def deleteCallback(self, feedback):
        for key in self._marker_map.keys():
            self.server.erase(key)
        self.server.applyChanges()
        self.tearDown()

    def tearDown(self, keepAlive=False):
        # forcefully shut down service before killing node
        if self._dserver:
            self._dserver.set_service.shutdown("User deleted template.")
        # for rostest (and potentially other cases), we want to clean up but keep the node alive
        if not keepAlive:
            rospy.signal_shutdown("User deleted template.")

    def processFeedback(self, feedback):
        self.server.applyChanges()

