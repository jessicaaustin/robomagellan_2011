#
# Read GPS data from Garmin device via gpsbabel, and extract out
#   lat, lng, vel, and heading
#
# Uses pyproj to convert geographic coords (lat,lng) to local map coords (x,y)
#   see http://code.google.com/p/pyproj/
#
# TODO: publish as ros topic
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

# constants
TEST_MODE = True
KNOT_TO_M_S = 0.514444444

# test data is currently zone 34, but chicago is zone 16
#projection = Proj({'proj':'utm','zone':16,'ellps':'WGS84'})
projection = Proj({'proj':'utm','zone':34,'ellps':'WGS84'})
init_x = None
init_y = None

def knot_to_vel(num):
  return KNOT_TO_M_S * num

def lat_lng_to_course_frame(lat,lng):
  return projection(lat/100,lng/100)

def publish_location(nmea_str, publisher):
  nmea_arr = nmea_str.split(",")
  lat = float(nmea_arr[3])
  lng = float(nmea_arr[5])
  vel = float(nmea_arr[7])
  heading = float(nmea_arr[8])

  global init_x
  global init_y
  if init_x is None:
    init_x,init_y = lat_lng_to_course_frame(lat,lng)

  x,y = lat_lng_to_course_frame(lat,lng)

  rospy.loginfo( "lat = %f" % lat ) 
  rospy.loginfo( "lng = %f" % lng )
  rospy.loginfo( "vel (knots) = %f" % vel )
  rospy.loginfo( "heading = %f" % heading )
  rospy.loginfo( "x (m, course frame) = %f" % (x - init_x) )
  rospy.loginfo( "y (m, course frame)) = %f" % (y - init_y) )
  rospy.loginfo( "vel (m/s) = %f" % knot_to_vel(vel) )
  rospy.loginfo( "" )


CMD = 'gpsbabel -T -i garmin -f usb: -o nmea -F -'
if TEST_MODE:
  CMD = os.getcwd() + '/../test/gpsbabel'

def main():
  pub = rospy.Publisher('odom', String)
  rospy.init_node('gps_location')
  p = Popen(CMD, stdout = PIPE, stderr = STDOUT, shell = True)
  while not rospy.is_shutdown():
    line = p.stdout.readline()
    if not line: 
      print 'nothing'
      break
    if "GPRMC" in line:
      publish_location(line, pub)

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException: pass
