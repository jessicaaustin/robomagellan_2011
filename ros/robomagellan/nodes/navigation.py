#!/usr/bin/env python
#
# Subscribes to:
#  /odom
#  /collision
# Publishes to:
#  /cmd_vel
#
# This node performs all calcuations in 'course frame':
#   origin (0, 0) is location of first GPS read
#   orientation goes from -pi to pi, (with robot nose initially pointing at theta=0):
#            y                    y                        
#       pi/2 ^               pi/2 ^             turnrate direction:
#            |                    |                   
#      pi    |    0         pi   ---   0         ---   ^ + 
#      <-----+----->x       <-- |   > -->x      |   >  | 
#      -pi   |    0         -pi  ---   0         ---
#            |-pi/2               |-pi/2   
#            v                    v             
#      course frame          robot frame ('>' = robot nose) 

import roslib; roslib.load_manifest('robomagellan')

import sys
import math
import threading

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from robomagellan.srv import *
from robomagellan.msg import *

import settings

# Understands how to move to a given point
# Runs on a thread, and continues until stop() is called on it
class Navigator(threading.Thread):
    def __init__(self):
        # setup thread listener
        super(Navigator, self).__init__()
        self._stop = threading.Event()
        # setup /cmd_vel publisher
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
        # start listening for the current position
        self.control_curr_pos = None
        rospy.Subscriber('odom', Odometry, self.setup_odom_callback())
        self.waypoint = None
        self.waypoint_threshold = None
        self.backwards = None

    def stop (self):
        self._stop.set()

    def stopped (self):
        return self._stop.isSet() or rospy.is_shutdown()

    def setup_odom_callback(self):
        def odom_callback(data):
            self.control_curr_pos = data
        return odom_callback

    def publish_cmd_vel(self, x, z):
        cmd = Twist()
        cmd.linear.x = x
        cmd.angular.z = z
        self.cmd_vel_pub.publish(cmd)

    def current_pos(self):
        x = self.control_curr_pos.pose.pose.position.x
        y = self.control_curr_pos.pose.pose.position.y
        q = self.control_curr_pos.pose.pose.orientation
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
        yaw = euler[2]
        return (x, y, yaw)

    def set_waypoint(self, waypoint, waypoint_threshold):
        self.waypoint = waypoint
        self.waypoint_threshold = waypoint_threshold

    def set_backwards(self, backwards):
        self.backwards = backwards

    # Travel from the current robot position to the given waypoint.
    def run(self):
        if self.backwards:
            self.move_backwards()
            return

        if self.waypoint == None or self.waypoint_threshold == None:
            rospy.logerr('set waypoint before running Navigator!')
            return

        # loop until we get a lock on our position
        while not self.stopped():
            if self.control_curr_pos != None:
                self.move_to_point(self.waypoint, self.waypoint_threshold)
                return
            else:
                # wait 1 second
                rospy.loginfo("Navigator: Waiting for current position...")
                rospy.sleep(1.0)

    # Travel from the current robot position to the given waypoint.
    # Uses proportional control for correction
    # TODO fix bug with controller--seems to be going unstable sometimes
    def move_to_point(self, waypoint, waypoint_threshold):
        rospy.loginfo("Navigator: Moving to location: (%s)", waypoint)

        # initial position
        xpos, ypos, thetapos = self.current_pos()
        xinit = xpos
        yinit = ypos

        xi = xpos
        yi = ypos

        # desired location
        xd = waypoint.coordinate.x
        yd = waypoint.coordinate.y
        td = math.atan2(yd-yi, xd-xi)

        # check and see if we're already there
        if (math.fabs(xpos-xd) < waypoint_threshold and
            math.fabs(ypos-yd) < waypoint_threshold):
            rospy.loginfo("Navigator: Point reached!")
            return

        # error in longitudinal distance (parallel to line to follow)
        xerr = 0
        # error in lateral distance (perpendicular to line)
        yerr = 0
        # angular error (yaw)
        terr = 0

        elapsedTime = settings.TIMESTEP

        # yerr = perpendicular distance from line connecting initial pos and desired pos
        # if the line going from initial point to desired point is defined as
        #   Ax + By + C = 0
        # or in another form (where m is the slope of the line)
        #   y - yd = m(x - xd)
        # then the distance is from point (m,n) to the line is (http://tinyurl.com/43s7mur)
        #   (Am + Bn + C)/sqrt(A*A+B*B)
        # where
        #   A = -m
        #   B = 1
        #   C = m*xd - yd
        slope = (yd - yinit)/(xd - xinit)
        A = -slope
        B = 1
        C = slope * xd - yd

        # FIRST, TURN ON THE SPOT
        while (math.fabs(thetapos - td) > settings.THETA_TOLERANCE and not self.stopped()):

            xpos, ypos, thetapos = self.current_pos()
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
            self.publish_cmd_vel(0.0, turnrate)

            # wait a timestep before recalculating gains
            elapsedTime+=settings.TIMESTEP
            rospy.sleep(settings.TIMESTEP)

        # NOW, MOVE TOWARDS POINT
        while ((math.fabs(xpos-xd) > waypoint_threshold or
                math.fabs(ypos-yd) > waypoint_threshold) and 
                not self.stopped()):

            xpos, ypos, thetapos = self.current_pos()

            xerr = math.sqrt( (xpos-xd)*(xpos-xd) + (ypos-yd)*(ypos-yd) )
            # see notes on yerr above
            yerr = (A*xpos + B*ypos + C)/(math.sqrt(A*A+B*B))

            rospy.logdebug("*** moving...")
            rospy.logdebug("pos=(%s, %s)", xpos, ypos)
            rospy.logdebug("des=(%s, %s)", xd, yd)
            rospy.logdebug("xerr=%s", xerr)
            rospy.logdebug("theta=%s", thetapos)
            rospy.logdebug("thdes=%s", td)
            rospy.logdebug("yerr=%s", yerr)

            speed =  settings.LAMBDA * xerr
            if (speed > settings.MAX_SPEED):
                speed = settings.MAX_SPEED

            turnrate = settings.ROBOT_LENGTH * (settings.A1 * yerr)
            # need to do this because of how we defined theta
            if (xd > xpos):
                turnrate *= -1 
            if (turnrate > settings.MAX_TURNRATE):
                turnrate = settings.MAX_TURNRATE
            if (turnrate < -settings.MAX_TURNRATE):
                turnrate = -settings.MAX_TURNRATE
            rospy.logdebug("turnrate=%s",turnrate)

            # move the robot
            self.publish_cmd_vel(speed, turnrate)

            # wait a timestep before recalculating gains
            elapsedTime+=settings.TIMESTEP
            rospy.sleep(settings.TIMESTEP)

        rospy.loginfo("Navigator: Point reached!")
        return

    # move backwards a bit. useful for re-orientation after encountering an obstacle
    def move_backwards(self):
        rospy.loginfo('Navigator: move_backwards')
        time_moved = 0
        while not self.stopped() and time_moved < settings.BACKWARDS_TIME:
            self.publish_cmd_vel(-1 * settings.MAX_SPEED, 0.0)
            time_moved += settings.TIMESTEP
            rospy.sleep(settings.TIMESTEP)
        self.publish_cmd_vel(0.0, 0.0)
        return
         

# Understands how to go to a given Waypoint, assuming no obstacles
# If the waypoint is a Cone waypoint, it will return control when it is sufficiently
# close to use the camera. Otherwise, it will return control when it has reached
# the waypoint.
class TraversalNavigator(threading.Thread):
    def __init__(self):
        rospy.loginfo('TraversalNavigator: init')

    # Go towards a waypoint until it's been reached
    def goToWaypoint(self, waypoint):
        rospy.loginfo('TraversalNavigator: moving to waypoint %s' % waypoint)
        waypoint_threshold = settings.WAYPOINT_THRESHOLD
        if waypoint.type == 'C':
            waypoint_threshold = settings.WAYPOINT_DIST

        # start moving towards the point, on another thread
        navigator = Navigator()
        navigator.set_waypoint(waypoint, waypoint_threshold)
        navigator.start()
        # return whenever the navigator is done
        navigator.join()


# Understands how to move around an obstacle, and back on a given path to a Waypoint
# TODO implement
class ObstacleAvoidanceNavigator():
    def __init__(self):
        return

# Understands how to come in contact with a Cone Waypoint
# TODO: integrate camera data into this
class WaypointCaptureNavigator():
    def __init__(self):
        rospy.loginfo('WaypointReached: init')
        self.collision = Collision()
        self.collision.forwardCollision = 'N'
        self.collision.backwardCollision = 'N'
        # start listening for a collision
        rospy.Subscriber("collision", Collision, self.create_collision_callback()) 
        return

    def create_collision_callback(self):
        def collision_callback(data):
            self.collision = data
        return collision_callback

    # Go towards a waypoint until the /collision topic reports a hit
    def goToWaypoint(self, waypoint):
        rospy.loginfo('WaypointCaptureNavigator: capturing waypoint %s' % waypoint)

        # start moving towards the point, on another thread
        navigator = Navigator()
        navigator.set_waypoint(waypoint, 0)
        navigator.start()
        
        # loop until we've reached the cone
        while not rospy.is_shutdown():
            if self.collision.forwardCollision or self.collision.backwardCollision:
                rospy.loginfo("WaypointCaptureNavigator: Cone reached")
                navigator.stop()
                rospy.sleep(1.0)
                
                # move backwards to clear ourselves of the cone
                navigator = Navigator()
                navigator.move_backwards()
                return
            else:
                rospy.sleep(0.1)

        # stop the navigator if we got a shutdown
        navigator.stop()
        return


# Moves according to given manual controls
# TODO implement
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

# TODO: shutdown cleanly upon kill 
def main():
    rospy.init_node('navigation') #, log_level=rospy.DEBUG)
    traversal_navigator = TraversalNavigator()
    capture_navigator = WaypointCaptureNavigator()

    response = ''
    while response != 'No waypoints remaining!':
        rospy.loginfo("Requesting next waypoint")
        waypoint = get_next_waypoint()
        traversal_navigator.goToWaypoint(waypoint)

        # waypoint reached! if it's a cone let's capture it
        if waypoint.type == 'C':
            capture_navigator.goToWaypoint(waypoint)

        rospy.loginfo("Marking waypoint as reached")
        response = waypoint_reached(waypoint)

    rospy.loginfo("Done.")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
