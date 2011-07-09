#!/usr/bin/env python

#
# For testing/demonstration purposes only
#

import roslib; roslib.load_manifest('robomagellan')

import rospy
from robomagellan.srv import *

def get_next_waypoint():
    rospy.wait_for_service('next_waypoint')
    try:
        next_waypoint = rospy.ServiceProxy('next_waypoint', GetNextWaypoint)
        resp = next_waypoint('')
        print resp
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def waypoint_reached():
    rospy.wait_for_service('waypoint_reached')
    try:
        next_waypoint = rospy.ServiceProxy('next_waypoint', GetNextWaypoint)
        waypoint_resp = next_waypoint('')
        waypoint_reached = rospy.ServiceProxy('waypoint_reached', WaypointReached)
        resp = waypoint_reached(waypoint_resp.waypoint)
        print resp
        return resp.message
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    resp = ''
    while resp != 'No waypoints remaining!':
        print "Requesting next waypoint"
        get_next_waypoint()
        print "Marking waypoint as reached"
        resp = waypoint_reached()
