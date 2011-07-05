#!/usr/bin/env python
#
# Subscribes to:
#  /odom
# Publishes to:
#  /cmd_vel
#  

import roslib; roslib.load_manifest('robomagellan')

import sys
import math

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

from robomagellan.srv import *
from robomagellan.msg import *

import settings

# Understands how to go to a given Waypoint, assuming no obstacles
# Uses proportional control for correction
class TraversalNavigator():
  def __init__(self):
    self.control_curr_pos = None
    return

  def setup_odom_callback(self):
    def odom_callback(data):
      self.control_curr_pos = data
    return odom_callback

  def goToWaypoint(self, waypoint):
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)

    # start listening for the current position
    rospy.Subscriber('odom', Odometry, self.setup_odom_callback())

    while not rospy.is_shutdown():
      if self.control_curr_pos != None:
        self.move_to_point(waypoint.coordinate.x, waypoint.coordinate.y, cmd_vel_pub)
        return
      else:
        # wait 1 second
        rospy.loginfo("Waiting for current position...")
        rospy.sleep(1.0)

  # TODO cleanup, remove duplication 
  # TODO use Waypoint instead
  def move_to_point(self,x,y,cmd_vel_pub):
      rospy.loginfo("Moving to location: (%d, %d)", x, y)

      # initial position
      xpos = self.control_curr_pos.pose.pose.position.x
      ypos = self.control_curr_pos.pose.pose.position.y
      thetapos = self.control_curr_pos.pose.pose.orientation.z
      xinit = xpos
      yinit = ypos

      xi = xpos
      yi = ypos

      # desired location
      xd = x
      yd = y
      td = math.atan2(yd-yi, xd-xi)/math.pi

      if (math.fabs(xpos-xd) < settings.WAYPOINT_THRESHOLD and math.fabs(ypos-yd) < settings.WAYPOINT_THRESHOLD):
          rospy.loginfo("Point reached!")
          return

      # error in lateral and longitudinal distances
      xerr = 0
      yerr = 0
      terr = 0

      elapsedTime = settings.TIMESTEP

      # Parameters for the line we're trying to follow
      slope = (yd - yinit)/(xd - xinit)
      B = 1
      A = -slope
      C = slope * xd - yd

      # FIRST, TURN ON THE SPOT
      while (math.fabs(thetapos - td) > settings.THETA_TOLERANCE):

          xpos = self.control_curr_pos.pose.pose.position.x
          ypos = self.control_curr_pos.pose.pose.position.y
          thetapos = self.control_curr_pos.pose.pose.orientation.z

          terr = td - thetapos

          rospy.logdebug("*** turning...")
          rospy.logdebug("pos=(%s, %s)", xpos, ypos)
          rospy.logdebug("theta=%s", thetapos)
          rospy.logdebug("thdes=%s", td)
          rospy.logdebug("therr=%s", terr)

          turnrate = settings.ROBOT_LENGTH*settings.A2*terr
          if (turnrate > settings.MAX_TURNRATE):
              turnrate = settings.MAX_TURNRATE
          if (turnrate < -settings.MAX_TURNRATE):
              turnrate = -settings.MAX_TURNRATE

          rospy.logdebug("turnrate=%s", turnrate)

          # move the robot
          cmd = Twist()
          cmd.linear.x = 0.0
          cmd.angular.z = turnrate
          cmd_vel_pub.publish(cmd)

          # wait a timestep before recalculating gains
          elapsedTime+=settings.TIMESTEP
          rospy.sleep(settings.TIMESTEP)


      # NOW, MOVE TOWARDS POINT
      while (math.fabs(xpos-xd) > settings.WAYPOINT_THRESHOLD or math.fabs(ypos-yd) > settings.WAYPOINT_THRESHOLD):

          distToPoint = math.sqrt( (xpos-xd)*(xpos-xd) + (ypos-yd)*(ypos-yd) )

          xpos = self.control_curr_pos.pose.pose.position.x
          ypos = self.control_curr_pos.pose.pose.position.y
          thetapos = self.control_curr_pos.pose.pose.orientation.z
   
          xerr = math.sqrt( (xpos - xd)*(xpos - xd) + (ypos - yd)*(ypos - yd) )
          yerr = (A*xpos + ypos + C)/(math.sqrt(A*A+B*B))
          td = math.atan2(yd - ypos, xd - xpos)/math.pi
          terr = td - thetapos

          rospy.logdebug("*** moving...")
          rospy.logdebug("pos=(%s, %s)", xpos, ypos)
          rospy.logdebug("des=(%s, %s)", xd, yd)
          rospy.logdebug("dist=%s", distToPoint)
          rospy.logdebug("theta=%s", thetapos)
          rospy.logdebug("thdes=%s", td)
          rospy.logdebug("therr=%s", terr)

          speed =  settings.LAMBDA * xerr
          if (speed > settings.MAX_SPEED):
              speed = settings.MAX_SPEED

          turnrate = -1 * settings.ROBOT_LENGTH * (settings.A1 * yerr - settings.A2 * terr)
          if (turnrate > settings.MAX_TURNRATE):
              turnrate = settings.MAX_TURNRATE
          if (turnrate < -settings.MAX_TURNRATE):
              turnrate = -settings.MAX_TURNRATE

          # move the robot
          cmd = Twist()
          cmd.linear.x = speed
          cmd.angular.z = turnrate
          cmd_vel_pub.publish(cmd)

          # wait a timestep before recalculating gains
          elapsedTime+=settings.TIMESTEP
          rospy.sleep(settings.TIMESTEP)

      rospy.loginfo("Point reached!")


# Understands how to move around an obstacle, and back on a given path to a Waypoint
class ObstacleAvoidanceNavigator():
  def __init__(self):
    return

# Understands how to come in contact with a Cone Waypoint
class WaypointCaptureNavigator():
  def __init__(self):
    return

# Moves according to given manual controls
class ManualControlNavigator():
  def __init__(self):
    return


def get_next_waypoint():
  rospy.wait_for_service('next_waypoint')
  try:
    next_waypoint_service = rospy.ServiceProxy('next_waypoint', GetNextWaypoint)
    resp = next_waypoint_service('')
    return resp.waypoint
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e

def waypoint_reached(waypoint):
  rospy.wait_for_service('waypoint_reached')
  try:
    waypoint_reached_service = rospy.ServiceProxy('waypoint_reached', WaypointReached)
    resp = waypoint_reached_service(waypoint)
    rospy.loginfo(resp)
    return resp.message
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e

def main():
  rospy.init_node('navigation')
  navigator = TraversalNavigator()

  response = ''
  while response != 'No waypoints remaining!':
    rospy.loginfo("Requesting next waypoint")
    waypoint = get_next_waypoint()
    navigator.goToWaypoint(waypoint)

    rospy.loginfo("Marking waypoint as reached")
    response = waypoint_reached(waypoint)

  rospy.loginfo("Done.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
