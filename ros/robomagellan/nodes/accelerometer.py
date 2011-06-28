#!/usr/bin/env python

import serial

class Accelerometer():

	def __init__(self, serialPort):
		"""
		for reading the X, Y and Z acceleration values from a SparkFun
		accelerometer
		"""

		self.serialPort = serial.Serial(port = serialPort,
										baudrate = 9600,
										timeout = 1)
		self.serialPort.setRTS(level = True)
		self.serialPort.setDTR(level = True)
		self.serialPort.flushInput()

		self.minimumThreshold = 0.1

	def getAcceleration(self):
		"""
		read the X, Y and Z values and return them as a tuple
		"""

		self.serialPort.flushInput()

		#
		# read whatever is left of the most recent message
		#
		characterRead = self.serialPort.read(1)
		while (characterRead != '\r'):
			characterRead = self.serialPort.read(1)

		#
		# and then read a whole message
		#
		characterRead = self.serialPort.read(1)
		inputMessageBuffer = characterRead
		while (characterRead != '\r'):
			characterRead = self.serialPort.read(1)
			inputMessageBuffer = inputMessageBuffer + characterRead

		try:
			xAcceleration = float(inputMessageBuffer[3:8])
			yAcceleration = float(inputMessageBuffer[12:17])
			zAcceleration = float(inputMessageBuffer[21:26])

		except:
			return (0.0, 0.0, 0.0, 'Unknown')

		if (abs(xAcceleration) > self.minimumThreshold):
			if (xAcceleration < 0.0):
				xOrient = 'Up'
			else:
				xOrient = 'Down'
		else:
			xOrient = 'Level'
		if (abs(yAcceleration) > self.minimumThreshold):
			if (yAcceleration < 0.0):
				yOrient = 'Up'
			else:
				yOrient = 'Down'
		else:
			yOrient = 'Level'
		if (abs(zAcceleration) > self.minimumThreshold):
			if (zAcceleration < 0.0):
				zOrient = 'Down'
			else:
				zOrient = 'Up'
		else:
			zOrient = 'Level'

		if (xOrient == 'Level' and yOrient == 'Level' and zOrient == 'Up'):
			orientation = 'FaceUp'
		elif (xOrient == 'Level' and yOrient == 'Level' and zOrient == 'Down'):
			orientation = 'FaceDown'
		elif (xOrient == 'Level' and yOrient == 'Up' and zOrient == 'Level'):
			orientation = 'Y-up'
		elif (xOrient == 'Up' and yOrient == 'Level' and zOrient =='Level'):
			orientation = 'X-up'
		elif (xOrient == 'Level' and yOrient == 'Down' and zOrient == 'Level'):
			orientation = 'Y-down'
		elif (xOrient == 'Down' and yOrient == 'Level' and zOrient == 'Level'):
			orientation = 'X-down'
		else:
			orientation = 'Intermediate'

		return (xAcceleration, yAcceleration, zAcceleration, orientation)

if __name__== '__main__':

	accelerometer = Accelerometer()

	while True:
		results = accelerometer.getAcceleration()

		if (results == None):
			print "No data available"

		print "X:%f, Y:%f, Z:%f, Orientation:%s\n" % (results[0], results[1], results[2], results[3])

