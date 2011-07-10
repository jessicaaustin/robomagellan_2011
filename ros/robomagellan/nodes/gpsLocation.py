#!/usr/bin/env python

# Read GPS data from Garmin device via gpsbabel, and extract out
#   lat, lng, vel, and heading
#
# Uses pyproj to convert geographic coords (lat,lng) to local map coords (x,y)
#   see http://code.google.com/p/pyproj/
#
# TODO: make this a class
# TODO: figure out LAT_OFFSET and LNG_OFFSET
#

import roslib; roslib.load_manifest('robomagellan')

import sys
import math
from subprocess import Popen, PIPE, STDOUT
from pyproj import Proj
import os

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import settings

# constants
KNOT_TO_M_S = 0.514444444

projection = Proj({'proj':'utm','zone':16,'ellps':'WGS84'})
init_x = None
init_y = None

def knot_to_vel(num):
    return KNOT_TO_M_S * num

def lat_lng_to_course_frame(lat,lng):
    return projection(lat,lng)

def publish_location(nmea_str, publisher):
    nmea_arr = nmea_str.split(",")
    lat = (float(nmea_arr[3]) / 100) + settings.LAT_OFFSET
    lng = (-1 * float(nmea_arr[5]) / 100) + settings.LNG_OFFSET
    vel = float(nmea_arr[7])
    heading = float(nmea_arr[8])

    rospy.loginfo('lat,lng=(%f,%f)' % (lat,lng))
    rospy.loginfo('vel=%f,heading=%f' % (vel,heading))

    global init_x
    global init_y
    if init_x is None:
        init_x,init_y = lat_lng_to_course_frame(lat,lng)

    x,y = lat_lng_to_course_frame(lat,lng)

    odom = Odometry()

    odom.pose.pose.position.x = x - init_x
    odom.pose.pose.position.y = y - init_y
    odom.twist.twist.linear.x = knot_to_vel(vel)
    # TODO convert to radians?
    odom.pose.pose.orientation.z = heading

    #rospy.loginfo(odom)
    publisher.publish(odom)


CMD = 'gpsbabel -T -i garmin -f usb: -o nmea -F -'
if settings.TEST_MODE:
    CMD = os.getcwd() + '/../test/gpsbabel'

def main():
    pub = rospy.Publisher('odom', Odometry)
    pub_raw = rospy.Publisher('gps_raw_data', String)
    rospy.init_node('gps_location')
    p = Popen(CMD, stdout = PIPE, stderr = STDOUT, shell = True)
    while not rospy.is_shutdown():
        line = p.stdout.readline()
        if not line:
            rospy.logerr("Could not read from GPS!")
            break
        pub_raw.publish(line)
        if "GPRMC" in line:
            publish_location(line, pub)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
