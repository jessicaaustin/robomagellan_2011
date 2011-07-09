#!/usr/bin/env python
import roslib; roslib.load_manifest('robomagellan')
import rospy
import serial
import string
from robomagellan.msg import Range

def readInteger(source):
    inputBuffer = ''

    characterRead = source.read(1)
    while (characterRead != '\n'):
        if (characterRead in string.digits):
            inputBuffer = inputBuffer + characterRead
        characterRead = source.read(1)

    rospy.loginfo('inputBuffer %d', int(inputBuffer))
    return int(inputBuffer)

def rangeFinder():
    rospy.init_node('rangeFinder', log_level=rospy.DEBUG)
    rospy.loginfo('rangeFinder started')
    publisher = rospy.Publisher('rangeTopic', Range)
    currentRange = Range('Forward', 1, 0)
    serialPort = serial.Serial(port = '/dev/ttyUSB2',
                                                       baudrate = 9600,
                                                       timeout = 1)
    serialPort.setRTS(level = True)
    serialPort.setDTR(level = True)

    serialPort.flushInput()

    while not rospy.is_shutdown():
        try:
            rospy.loginfo('reading ...')
            currentRange.rangeInCm = readInteger(serialPort)

        except:
            rospy.loginfo('Unable to get range data')

        try:
            publisher.publish(currentRange)

        except:
            rospy.loginfo('Unable to publish range data')

        rospy.sleep(5.0)

if __name__== '__main__':
    try:
        rangeFinder()

    except rospy.ROSInterruptException:
        rospy.loginfo('rangeFinder stopping')
