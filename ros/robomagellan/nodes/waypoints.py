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
from robomagellan.srv import GetNextWaypoint
from robomagellan.msg import Waypoint

import rospy
from geometry_msgs.msg import Point

class WaypointServer():
  def __init__(self):
    self.waypoints = self.read_waypoints_from_file()
    self.current_index = 0
    return

  def current_waypoint(self):
    return self.waypoints[self.current_index]

  # TODO actually read from file
  def read_waypoints_from_file(self):
    return [Waypoint('P', Point(1.0, 1.0, 0)), 
            Waypoint('C', Point(5.0, 10.0, 0)), 
            Waypoint('P', Point(25.0, 10.0, 0)), 
            Waypoint('P', Point(12.0, 25.0, 0))]
 

def handle_next_waypoint(server):
  def get_current_waypoint_from_server(request):
    return server.current_waypoint()
  return get_current_waypoint_from_server

def waypoints_server():
  rospy.init_node('waypoints_server')
  server = WaypointServer()
  s = rospy.Service('next_waypoint', GetNextWaypoint, handle_next_waypoint(server))
  rospy.loginfo('ready')
  rospy.spin()

if __name__ == "__main__":
  waypoints_server()
