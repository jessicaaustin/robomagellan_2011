#!/usr/bin/env python
import roslib; roslib.load_manifest('robomagellan')

import rospy
from robomagellan.srv import *

def waypoints_client():
  rospy.wait_for_service('next_waypoint')
  try:
    next_waypoint = rospy.ServiceProxy('next_waypoint', GetNextWaypoint)
    resp = next_waypoint('')
    print resp
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e

if __name__ == "__main__":
  print "Requesting next waypoint" 
  waypoints_client()
