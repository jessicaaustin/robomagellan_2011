#!/usr/bin/env python

#
# Monitors the /odom topic and waypoints service, and publishes
# to /collision topic whenever it is close to a cone waypoint
# 
# For simulation purposes only
#

import roslib; roslib.load_manifest('robomagellan')

import os, sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../nodes/")

import rospy
from robomagellan.msg import *
from robomagellan.srv import *

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

import math
import settings

class CollisionMock():
  def __init__(self):
    self.current_pos = None

  def get_next_waypoint(self):
    rospy.wait_for_service('next_waypoint')
    try:
      next_waypoint = rospy.ServiceProxy('next_waypoint', GetNextWaypoint)
      return next_waypoint('').waypoint
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

  def setup_odom_callback(self):
    def odom_callback(data):
      self.current_pos = data.pose.pose.position
    return odom_callback

  def check_for_collision(self):
    if self.current_pos != None:
      waypoint = self.get_next_waypoint()
      if waypoint.type == 'C':
        if (math.fabs(self.current_pos.x-waypoint.coordinate.x) < settings.COLLISION_THRESHOLD and 
            math.fabs(self.current_pos.y-waypoint.coordinate.y) < settings.COLLISION_THRESHOLD):
          rospy.loginfo('COLLISION!')
    else:
      rospy.loginfo('Waiting for current position')

if __name__ == "__main__":
  rospy.init_node('collision_node')
  collision_mock = CollisionMock()
  rospy.Subscriber('odom', Odometry, collision_mock.setup_odom_callback())
  while not rospy.is_shutdown():
    collision_mock.check_for_collision()
    rospy.sleep(1.0)

