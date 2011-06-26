"""
 This class accepts Movement commands and sends the relevant commands
 to the individual controllers.
"""


import serial
import threading
import Queue
import time

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
			movement is expected to contain the three values from the Move message
		"""
		leftWheel, rightWheel, rampUp = movement

		print "move() %d, %d, %d" % (leftWheel, rightWheel, rampUp)

		# place the appropriate commands on each controllerQ
		synchronizer.clear()
		self.leftQ.put((leftWheel, rampUp))
		self.rightQ.put((-(rightWheel), rampUp))
		synchronizer.set()

		return

class MotionMind(threading.Thread):
	def __init__(self, name, serialPort, address, commandQueue):
		threading.Thread.__init__(self)

		self.name = name
		self.commandQueue = commandQueue
		self.currentVelocity = 0
		self.intervalDelay = 1.0
		self.serialPort = serial.Serial(port = serialPort,
										baudrate = 9600,
										timeout = 1)
		self.serialPort.setRTS(level = True)
		self.serialPort.setDTR(level = True)
		self.address = address
		# set all of the PID parameters
		self.sendCommand("W%s 04 10000" % (self.address))# P gain
		self.sendCommand("W%s 05 0" % (self.address))    # I gain
		self.sendCommand("W%s 06 0" % (self.address))    # D gain 
		self.sendCommand("W%s 08 10000" % (self.address))# PID scalar
		
		return

	def checkForCollision(self):
		return

	def run(self):
		while True:
			newVelocity, rampUpSeconds = self.commandQueue.get(True)

			synchronizer.wait()

			velocityChange = newVelocity - self.currentVelocity
			velocityIncrement = int(velocityChange / rampUpSeconds)
			intervals = rampUpSeconds

			print "%s: %d, %d, %d, %d" % (self.name, self.currentVelocity, velocityChange, velocityIncrement, intervals)
		
			while (intervals > 0):
				self.checkForCollision()
				self.currentVelocity = self.currentVelocity + velocityIncrement
				print "+%s: %d, %d, %d, %d" % (self.name, self.currentVelocity, velocityChange, velocityIncrement, intervals)
				command = "V%s %d" % (self.address, self.currentVelocity)
				self.sendCommand(command)
				intervals = intervals - 1
				if (intervals > 0):
					time.sleep(self.intervalDelay)
				self.checkForCollision()

			if (newVelocity != self.currentVelocity):
				command = "V%s %d" % (self.address, newVelocity)
				self.sendCommand(command)
				self.currentVelocity = newVelocity
				self.checkForCollision()

	def sendCommand(self, command):

		self.serialPort.write("%s\r\n" % (command))
		result = self.serialPort.read(20)

		return

