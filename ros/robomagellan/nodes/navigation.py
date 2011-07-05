#!/usr/bin/env python
#
# Subscribes to:
#  /odom
# Publishes to:
#  /cmd_vel
#  
# TODO: take in point to go to via ROS service instead of using hard-coded value
# TODO: publish a reasonable amount of log messages
# TODO: use robomagellan.msg/Waypoint
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
  # TODO move magic numbers into settings file so they can be easily tweaked
  # TODO use Waypoint instead
  def move_to_point(self,x,y,cmd_vel_pub):
      rospy.loginfo("Moving to location: (%d, %d)", x, y)

      # constants
      TIMESTEP = .05 #seconds
      TOLERANCE = .05 #meters
      THETA_TOLERANCE = .05 #radians

      # length of the robot
      LENGTH = .5 #meters

      # maximum speed and turnrate allowed for the robot
      MAX_SPEED = 1 #m/s
      MAX_TURNRATE = 1 #rad/s

      # control parameters (for point-to-point controller)
      LAMBDA = .2
      A1 = 2
      A2 = 1

      # parameters for GVG
      GRID_SIZE = .5
      DIST_TOLERANCE = math.sqrt(2)*GRID_SIZE

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

      if (math.fabs(xpos-xd) < TOLERANCE and math.fabs(ypos-yd) < TOLERANCE):
          rospy.loginfo("Point reached!")
          return

      # error in lateral and longitudinal distances
      xerr = 0
      yerr = 0
      terr = 0

      elapsedTime = TIMESTEP

      # Parameters for the line we're trying to follow
      slope = (yd - yinit)/(xd - xinit)
      B = 1
      A = -slope
      C = slope * xd - yd

      # output parameters
      #double speed, turnrate
      #double distToPoint, distToObj

      # FIRST, TURN ON THE SPOT
      while (math.fabs(thetapos - td) > THETA_TOLERANCE):

          xpos = self.control_curr_pos.pose.pose.position.x
          ypos = self.control_curr_pos.pose.pose.position.y
          thetapos = self.control_curr_pos.pose.pose.orientation.z

          terr = td - thetapos

          rospy.logdebug("*** turning...")
          rospy.logdebug("pos=(%s, %s)", xpos, ypos)
          rospy.logdebug("theta=%s", thetapos)
          rospy.logdebug("thdes=%s", td)
          rospy.logdebug("therr=%s", terr)

          turnrate = LENGTH*A2*terr
          if (turnrate > MAX_TURNRATE):
              turnrate = MAX_TURNRATE
          if (turnrate < -MAX_TURNRATE):
              turnrate = -MAX_TURNRATE

          rospy.logdebug("turnrate=%s", turnrate)

          # move the robot
          cmd = Twist()
          cmd.linear.x = 0.0
          cmd.angular.z = turnrate
          cmd_vel_pub.publish(cmd)

          # wait a timestep before recalculating gains
          elapsedTime+=TIMESTEP
          rospy.sleep(TIMESTEP)


      # NOW, MOVE TOWARDS POINT
      while (math.fabs(xpos-xd) > TOLERANCE or math.fabs(ypos-yd) > TOLERANCE):

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

          speed =  LAMBDA * xerr
          if (speed > MAX_SPEED):
              speed = MAX_SPEED

          turnrate = -1 * LENGTH * (A1 * yerr - A2 * terr)
          if (turnrate > MAX_TURNRATE):
              turnrate = MAX_TURNRATE
          if (turnrate < -MAX_TURNRATE):
              turnrate = -MAX_TURNRATE

          # move the robot
          cmd = Twist()
          cmd.linear.x = speed
          cmd.angular.z = turnrate
          cmd_vel_pub.publish(cmd)

          # wait a timestep before recalculating gains
          elapsedTime+=TIMESTEP
          rospy.sleep(TIMESTEP)

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


def main():
  rospy.init_node('navigation')
  navigator = TraversalNavigator()
  waypoint = Waypoint('C', Point(1, 1, 0))  
  navigator.goToWaypoint(waypoint)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
