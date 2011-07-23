#!/usr/bin/env python

#
# Reads in list of obstacles from a world file. Then monitors the /odom topic 
# and publishes to /collision topic whenever it is close to an obstacle
#
# For simulation purposes only
#

import roslib; roslib.load_manifest('robomagellan')

import os, sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../nodes/")

import rospy
from robomagellan.msg import *
from robomagellan.srv import *
from CollisionPublisher import *

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

import math
import settings

# reads in obstacles from a stage world file
class WorldFileReader():
    def __init__(self):
        return
    
    def read_obstacles(self, filename):
        rospy.loginfo('Reading world obstacles from file %s' % filename)
        obstacles = []
        file = open(filename)
        line = file.readline()
        while len(line) > 0:
            if (line.startswith("cone")):
                obstacle_coords = line.split("[")[1].split("]")[0].split()
                obstacles.append(Waypoint('C', Point(float(obstacle_coords[0]), float(obstacle_coords[1]), 0.0)))
            line = file.readline() 
        return obstacles


# Publishes to collision topic if we're sufficiently close to a cone
class CollisionMock():
    def __init__(self, obstacles):
        self.obstacles = obstacles
        self.current_pos = None
        self.collision_publisher = CollisionPublisher()
        rospy.loginfo('CollisionMock: initializing with obstacles:\n %s' % self.obstacles)

    def setup_odom_callback(self):
        def odom_callback(data):
            self.current_pos = data.pose.pose.position
        return odom_callback

    def check_for_collision(self):
        if self.current_pos != None:
            for obstacle in obstacles:
                if (obstacle.type == 'C' and 
                        math.fabs(self.current_pos.x-obstacle.coordinate.x) < settings.COLLISION_THRESHOLD and
                        math.fabs(self.current_pos.y-obstacle.coordinate.y) < settings.COLLISION_THRESHOLD):
                    rospy.loginfo('COLLISION!')
                    self.collision_publisher.publish_collision(True, False)
                else:
                    self.collision_publisher.publish_collision(False, False)
        else:
            rospy.loginfo('Waiting for current position')

if __name__ == "__main__":
    if len(sys.argv) < 2 or not os.path.exists(sys.argv[1]) or not os.path.isfile(sys.argv[1]):
        rospy.logerr('Must supply world file!')
        sys.exit(1)
    else:
        world_file = sys.argv[1]
    reader = WorldFileReader()
    obstacles = reader.read_obstacles(world_file)

    rospy.init_node('collision_mock')
    collision_mock = CollisionMock(obstacles)
    rospy.Subscriber('odom', Odometry, collision_mock.setup_odom_callback())
    while not rospy.is_shutdown():
        collision_mock.check_for_collision()
        rospy.sleep(0.5)
