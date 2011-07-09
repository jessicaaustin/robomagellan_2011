#!/usr/bin/env python
import roslib; roslib.load_manifest('robomagellan')
import rospy
from robomagellan.msg import Acceleration
from accelerometer import *

def acceleration():
    rospy.init_node('acceleration', log_level=rospy.DEBUG)
    rospy.loginfo('acceleration started')
    publisher = rospy.Publisher('accelerationTopic', Acceleration)

    currentAcceleration = Acceleration()

    acceleration = Accelerometer('/dev/ttyUSB3')

    while not rospy.is_shutdown():
        try:
            currentAcceleration.xAcceleration, currentAcceleration.yAcceleration, currentAcceleration.zAcceleration, currentAcceleration.orientation = acceleration.getAcceleration()

        except:
            rospy.loginfo('Unable to get acceleration data')

        try:
            publisher.publish(currentAcceleration)

        except:
            rospy.loginfo('Unable to publish acceleration data')

if __name__== '__main__':
    try:
        acceleration()

    except rospy.ROSInterruptException:
        rospy.loginfo('acceleration stopping')
