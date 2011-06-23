#
# Read GPS data from Garmin device via gpsbabel, and extract out
#   lat, lng, vel, and heading
#
# Uses pyproj to convert geographic coords (lat,lng) to local map coords (x,y)
#   see http://code.google.com/p/pyproj/
#
# TODO: Convert these values to course frame and SI units
#

from subprocess import Popen, PIPE, STDOUT
from pyproj import Proj
import os

# constants
TEST_MODE = True
KNOT_TO_M_S = 0.514444444

# test data is currently zone 34, but chicago is zone 16
#projection = Proj({'proj':'utm','zone':16,'ellps':'WGS84'})
projection = Proj({'proj':'utm','zone':34,'ellps':'WGS84'})

def knot_to_vel(num):
  return KNOT_TO_M_S * num

def lat_lng_to_course_frame(lat,lng):
  return projection(lat/100,lng/100)

CMD = 'gpsbabel -T -i garmin -f usb: -o nmea -F -'
if TEST_MODE:
  CMD = os.getcwd() + '/../test/gpsbabel'

firstLineFound = False

p = Popen(CMD, stdout = PIPE, stderr = STDOUT, shell = True)
while True:
        line = p.stdout.readline()
        if not line: 
          print 'nothing'
          break
        if "GPRMC" in line:
                nmea_arr = line.split(",")
                lat = float(nmea_arr[3])
                lng = float(nmea_arr[5])
                vel = float(nmea_arr[7])
                heading = float(nmea_arr[8])
                
                if not firstLineFound:
                  init_x,init_y = lat_lng_to_course_frame(lat,lng)
                  firstLineFound = True

                x,y = lat_lng_to_course_frame(lat,lng)
        
                print "lat = %f" % lat
                print "lng = %f" % lng
                print "vel (knots) = %f" % vel
                print "heading = %f" % heading
                print "x (m, course frame) = %f" % (x - init_x)
                print "y (m, course frame)) = %f" % (y - init_y) 
                print "vel (m/s) = %f" % knot_to_vel(vel)
                print ""


