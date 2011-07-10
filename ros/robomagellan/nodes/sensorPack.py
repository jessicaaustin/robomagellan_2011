#!/usr/bin/env python
import roslib; roslib.load_manifest('robomagellan')
import rospy
import serial
import string
import sys
import time
from robomagellan.msg import Range
from robomagellan.msg import Gyro

currentRange = Range(rangeInCm = 0)
currentGyro = Gyro(rotationRate = 0, rotationDirection = 'N')
rangePublisher = rospy.Publisher('rangeTopic', Range)
gyroPublisher = rospy.Publisher('gyroTopic', Gyro)
serialPort = serial.Serial(port = '/dev/ttyUSB2',
                                                   baudrate = 9600,
                                                   timeout = 1)
serialPort.setRTS(level = True)
serialPort.setDTR(level = True)
serialPort.flushInput()

def processNextSensorMessage(source):

    # first read the next message
    characterRead = source.read(1)
    inputMessageBuffer = characterRead
    while (characterRead != '\n'):
        characterRead = source.read(1)
        inputMessageBuffer = inputMessageBuffer + characterRead
    if inputMessageBuffer[-1] == '\r':
        inputMessageBuffer = inputMessageBuffer[:-1]

    # then extract the fields
    messageFields = inputMessageBuffer.split('|')

    if messageFields[0] == 'Se':
        return

    elif messageFields[0] == '0':
        return

    elif messageFields[0] == 'Gy':
        if len(messageFields) == 3:
            currentGyro.rotationRate = int(messageFields[1])
            currentGyro.rotationDirection = messageFields[2]
            if (currentGyro.rotationDirection == 'N' or currentGyro.rotationRate < 2):
                return

            try:
                gyroPublisher.publish(currentGyro)

            except:
                print sys.exc_info()[0]
                rospy.logerr('Unable to publish Gyro message')

    elif messageFields[0] == 'Rg':
        if len(messageFields) == 2:
            currentRange.rangeInCm = int(messageFields[1])

            try:
                rangePublisher.publish(currentRange)

            except:
                print sys.exc_info()[0]
                rospy.logerr('Unable to publish Range message')

    else:
        rospy.logerr('Unable to parse sensor message')

    return

def sensorPack():
    rospy.init_node('sensorPack', log_level=rospy.DEBUG)
    rospy.loginfo('sensorPack started')

    while not rospy.is_shutdown():
        try:
            processNextSensorMessage(serialPort)

        except:
            print sys.exc_info()[0]
            rospy.logerr('Unable to get sensor data')

        time.sleep(0.5)

if __name__== '__main__':
    try:
        sensorPack()

    except rospy.ROSInterruptException:
        rospy.loginfo('sensorPack stopping')
