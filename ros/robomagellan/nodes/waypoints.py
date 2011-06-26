#!/usr/bin/env python
#
# Service that will list the next waypoint to go to
# 
# TODO: implement
#

import roslib; roslib.load_manifest('robomagellan')
from robomagellan.srv import GetNextWaypoint

import rospy
from geometry_msgs.msg import Point

def handle_next_waypoint(req):
  return Point(1,2,0)

def waypoints_server():
  rospy.init_node('waypoints_server')
  s = rospy.Service('next_waypoint', GetNextWaypoint, handle_next_waypoint)
  rospy.loginfo('ready')
  rospy.spin()

if __name__ == "__main__":
  waypoints_server()
