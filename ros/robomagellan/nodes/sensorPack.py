#!/usr/bin/env python
import roslib; roslib.load_manifest('robomagellan')
import rospy
import serial
import string
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
	inputMessageBuffer = ''

	# first read the next message
	characterRead = source.read(1)
	while (characterRead != '\n'):
		inputMessageBuffer = inputMessageBuffer + source.read(1)

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

			gyroPublisher.publish(currentGyro)
		
	elif messageFields[0] == 'Rg':
		if len(messageFields) == 2:
			currentRange.rangeInCm = int(messageFields[1])
			if currentRange.rangeInCm > 110:
				return

			rangePublisher.publish(currentRange)
		
	else:
		rospy.loginfo('Unable to parse sensor message')

	return

def sensorPack():
	rospy.init_node('sensorPack', log_level=rospy.DEBUG)
	rospy.loginfo('sensorPack started')

	while not rospy.is_shutdown():
		try:
			processNextSensorMessage()
		
		except:
			rospy.loginfo('Unable to get sensor data')

if __name__== '__main__':
	try:
		sensorPack()

	except rospy.ROSInterruptException:
		rospy.loginfo('sensorPack stopping')
