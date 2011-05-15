#
# Pan and tilt a camera
#


import time
from SSC32 import *

class PanTilt:
	def __init__(self, serialPort):
		self.cameraPanTilt = SSC32(serialPort)
		self.panServo = 0
		self.tiltServo = 4
		self.defaultSpeed = 300
		self.panCenter = "#%d P1500 S%d" % (self.panServo, self.defaultSpeed)
		self.tiltCenter = "#%d P1500 S%d" % (self.tiltServo, self.defaultSpeed)
		

	def centerCamera(self):
		self.cameraPanTilt.commandNoResponse(self.panCenter)
		self.cameraPanTilt.commandNoResponse(self.tiltCenter)

		return

	def exerciseCamera(self):
		self.centerCamera()
		time.sleep(1.0)
		self.tiltCamera(20)
		time.sleep(1.0)
		self.panCamera(20)
		time.sleep(1.0)
		self.panCamera(-20)
		time.sleep(1.0)
		self.tiltCamera(-20)
		time.sleep(1.0)
		self.centerCamera()
		
		return

	def tiltCamera(self, tiltFactor):
		if (tiltFactor < -50):
			tiltFactor = -50
		elif (tiltFactor > 50):
			tiltFactor = 50

		tilt = tiltFactor * 15 + 1500
		self.cameraPanTilt.commandNoResponse("#%d P%d S%d" % (self.tiltServo, tilt, self.defaultSpeed))

		return

	def panCamera(self, panFactor):
		if (panFactor < -50):
			panFactor = -50
		elif (panFactor > 30):
			panFactor = 30

		pan = panFactor * 15 + 1500
		self.cameraPanTilt.commandNoResponse("#%d P%d S%d" % (self.panServo, pan, self.defaultSpeed))

		return

	def aimCamera(self, pan, tilt):
		self.panCamera(pan)
		self.tiltCamera(tilt)

		return
