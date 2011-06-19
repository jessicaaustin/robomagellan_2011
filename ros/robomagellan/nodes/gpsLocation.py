#
# Read GPS data from Garmin device via gpsbabel, and extract out
#   lat, lng, vel, and heading
#
# TODO: Convert these values to course frame and SI units
# TODO: publish information on ROS topic
#

from subprocess import Popen, PIPE, STDOUT

p = Popen('gpsbabel -T -i garmin -f usb: -o nmea -F -', stdout = PIPE, stderr = STDOUT, shell = True)
while True:
        line = p.stdout.readline()
        if not line: break
        if "GPRMC" in line:
                nmea_arr = line.split(",")
                lat = float(nmea_arr[3])
                lng = float(nmea_arr[5])
                vel = float(nmea_arr[7])
                heading = float(nmea_arr[8])
                print "lat = %f" % lat
                print "lng = %f" % lng
                print "vel = %f" % vel
                print "heading = %f" % heading

