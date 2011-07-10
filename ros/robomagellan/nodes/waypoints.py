#!/usr/bin/env python
#
# Service that will list the next waypoint to go to
#
#   Reads in a list of ORDERED gps waypoints from a file
#   Service: /next_waypoint (GetNextWaypoint.srv)
#
# for a usage example, see test/waypoints_client.py
#

import roslib; roslib.load_manifest('robomagellan')
from robomagellan.srv import *
from robomagellan.msg import *

import rospy
from geometry_msgs.msg import Point

import os
import sys

class WaypointServer():
    def __init__(self):
        self.current_index = 0
        return

    def set_waypoints(self, waypoints):
        self.waypoints = waypoints

    def current_waypoint(self):
        return self.waypoints[self.current_index]

    def waypoint_reached(self, waypoint):
        rospy.loginfo('marking waypoint as reached: %s' % waypoint)
        if self.current_index == (len(self.waypoints) - 1):
            return "No waypoints remaining!"
        self.current_index += 1
        return ("Waypoint reached! %d waypoints remaining" % (len(self.waypoints) - self.current_index))


class WaypointFileReader():
    def __init__(self):
        return
    
    # TODO convert from GPS to course frame when reading from file
    def read_file(self, filename):
        rospy.loginfo('Reading waypoints from file %s' % filename)
        waypoints = []
        file = open(filename)
        line = file.readline()
        while len(line) > 0:
            args = line.split()
            waypoints.append(Waypoint(args[0], Point(float(args[1]), float(args[2]), 0.0)))
            line = file.readline() 
        rospy.loginfo('Waypoints: \n%s' % waypoints)
        return waypoints


def handle_waypoint_reached(server):
    def send_message_to_server(request):
        return server.waypoint_reached(request.waypoint)
    return send_message_to_server

def handle_next_waypoint(server):
    def get_current_waypoint_from_server(request):
        return server.current_waypoint()
    return get_current_waypoint_from_server

def waypoints_server(waypoints_file):
    rospy.init_node('waypoints_server')
    server = WaypointServer()
    waypoints = WaypointFileReader().read_file(waypoints_file)
    server.set_waypoints(waypoints)
    rospy.Service('next_waypoint', GetNextWaypoint, handle_next_waypoint(server))
    rospy.Service('waypoint_reached', WaypointReached, handle_waypoint_reached(server))
    rospy.loginfo('ready')
    rospy.spin()

if __name__ == "__main__":
    if len(sys.argv) < 2 or not os.path.exists(sys.argv[1]) or not os.path.isfile(sys.argv[1]):
        rospy.logerr('Must supply waypoints file!')
    else:
        waypoints_file = sys.argv[1]
        waypoints_server(waypoints_file)
