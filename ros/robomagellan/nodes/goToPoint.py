#!/usr/bin/env python
#
# goes to a given point, using proportional control for correction
#  
# TODO: cleanup code, remove duplication, don't use globals, etc
# TODO: take in point to go to via ROS service instead of using hard-coded value
# TODO: publish a reasonable amount of log messages
#
import roslib; roslib.load_manifest('robomagellan')

import sys
import math

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry

# TODO don't use globals
control_curr_pos = None

# pick up the current position
def odom_callback(data):
    global control_curr_pos
    control_curr_pos = data

# move to a given point
# TODO cleanup, remove duplication 
def move_to_point(x,y,cmd_vel_pub):
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
    xpos = control_curr_pos.pose.pose.position.x
    ypos = control_curr_pos.pose.pose.position.y
    thetapos = control_curr_pos.pose.pose.orientation.z
    xinit = xpos
    yinit = ypos

    xi = xpos
    yi = ypos

    # desired location
    xd = x
    yd = y
    td = math.atan2(yd-yi, xd-xi)/math.pi

    if (math.fabs(xpos-xd) < TOLERANCE and math.fabs(ypos-yd) < TOLERANCE):
        print "Point reached!"
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

        xpos = control_curr_pos.pose.pose.position.x
        ypos = control_curr_pos.pose.pose.position.y
        thetapos = control_curr_pos.pose.pose.orientation.z

        terr = td - thetapos

        rospy.loginfo("*** turning...")
        rospy.loginfo("pos=(%s, %s)", xpos, ypos)
        rospy.loginfo("theta=%s", thetapos)
        rospy.loginfo("thdes=%s", td)
        rospy.loginfo("therr=%s", terr)

        turnrate = LENGTH*A2*terr
        if (turnrate > MAX_TURNRATE):
            turnrate = MAX_TURNRATE
        if (turnrate < -MAX_TURNRATE):
            turnrate = -MAX_TURNRATE

        rospy.loginfo("turnrate=%s", turnrate)

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

        xpos = control_curr_pos.pose.pose.position.x
        ypos = control_curr_pos.pose.pose.position.y
        thetapos = control_curr_pos.pose.pose.orientation.z
 
        xerr = math.sqrt( (xpos - xd)*(xpos - xd) + (ypos - yd)*(ypos - yd) )
        yerr = (A*xpos + ypos + C)/(math.sqrt(A*A+B*B))
        td = math.atan2(yd - ypos, xd - xpos)/math.pi
        terr = td - thetapos

        rospy.loginfo("*** moving...")
        rospy.loginfo("pos=(%s, %s)", xpos, ypos)
        rospy.loginfo("des=(%s, %s)", xd, yd)
        rospy.loginfo("dist=%s", distToPoint)
        rospy.loginfo("theta=%s", thetapos)
        rospy.loginfo("thdes=%s", td)
        rospy.loginfo("therr=%s", terr)

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

    print "Point reached!"


def main():
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
    rospy.init_node('control')

    # start listening for the current position
    global control_curr_pos
    rospy.Subscriber('odom', Odometry, odom_callback)

    while not rospy.is_shutdown():

        if control_curr_pos != None:
            # move the robot using algorithm
            move_to_point(1.0, 1.0, cmd_vel_pub)
            move_to_point(5.0, 10.0, cmd_vel_pub)  
            move_to_point(25.0, 10.0, cmd_vel_pub)
            move_to_point(12.0, 25.0, cmd_vel_pub)
            exit()

#            # move the robot manually
#            cmd = Twist()
#            cmd.linear.x = -1.0
#            cmd.angular.z = 0.0
#            cmd_vel_pub.publish(cmd)

        # wait 1 second
        rospy.loginfo("Waiting for current position...")
        rospy.sleep(1.0)
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
