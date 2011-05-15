"""
 This class accepts Movement commands and sends the relevant commands
 to the individual controllers.
"""


import serial
import threading
import Queue

synchronizer = threading.Event()

class Mover():
	def __init__(self):
		"""
		 start a thread for each MotionMind controller and then
		 enter a loop reading commands, sending them to the controller
		 threads, synchronizing the start of the threads
		"""
		synchronizer.clear()
		self.leftQ  = Queue.Queue()
		self.rightQ = Queue.Queue()

		self.leftController = MotionMind("left", '/dev/ttyUSB0', '02', self.leftQ)
		self.rightController = MotionMind("right", '/dev/ttyUSB1', '01', self.rightQ)

		self.leftController.start()
		self.rightController.start()

		return

	def move(self, movement):

		"""
			movement is interpreted as a 2-tuple, containing a velocity
			for each of the left and right wheels
		"""
		leftVelocity, rightVelocity = movement

		# place the appropriate commands on each controllerQ
		synchronizer.clear()
		self.leftQ.put(leftVelocity)
		self.rightQ.put(-(rightVelocity))
		synchronizer.set()

		return

	def feedback(self, movementFeedback):
		"""
		 provide feedback about the last Move command. movementFeedback
		 is a 2-tuple containing the actual heading and the actual
		 speed.
		"""
		self.actualHeading, self.actualSpeed = movementFeedback

		return

	def abort(self):
		"""
		 stop all motion immediately
		"""

		return

	
class MotionMind(threading.Thread):
	def __init__(self, name, serialPort, address, commandQueue):
		threading.Thread.__init__(self)

		self.name = name
		self.commandQueue = commandQueue
		self.serialPort = serial.Serial(port = serialPort,
										baudrate = 9600,
										timeout = 1)
		self.serialPort.setRTS(level = True)
		self.serialPort.setDTR(level = True)
		self.address = address
		self.forwardLimit = False
		self.reverseLimit = False

		# set all of the PID parameters
		self.sendCommand("W%s 04 10000" % (self.address))# P gain
		self.sendCommand("W%s 05 0" % (self.address))    # I gain
		self.sendCommand("W%s 06 0" % (self.address))    # D gain 
		self.sendCommand("W%s 08 10000" % (self.address))# PID scalar
		
		return

	def run(self):
		while True:
			velocity = self.commandQueue.get(True)

			self.checkForLimit()

			if (velocity > 0 and self.forwardLimit):
				return

			if (velocity < 0 and self.reverseLimit):
				return
		
			# write command to controller and read acknowledgement
			command = "V%s %d" % (self.address, velocity)
			print self.name, command
			synchronizer.wait()
			self.sendCommand(command)

	def checkForLimit(self):
		return

	def sendCommand(self, command):

		self.serialPort.write("%s\r\n" % (command))
		result = self.serialPort.read(20)
		print self.address, result

		return

