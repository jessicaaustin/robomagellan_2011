"""
 This class accepts Movement commands and sends the relevant commands
 to the individual controllers.
"""

import roslib; roslib.load_manifest('robomagellan')

import rospy
from std_msgs.msg import String

import serial
import threading
import Queue
import time

synchronizer = threading.Event()

class Mover():
    def __init__(self):
        rospy.logdebug("initialzing Mover")
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

    def checkForwardCollision(self):
        if (self.leftController.forwardCollided or self.leftController.forwardCollision or
                self.rightController.forwardCollided or self.rightController.forwardCollision):
                self.leftController.resetForwardCollided()
                self.rightController.resetForwardCollided()
                return True
        else:
                return False
                
    def checkBackwardCollision(self):
        if (self.leftController.backwardCollided or self.leftController.backwardCollision or
                self.rightController.backwardCollided or self.rightController.backwardCollision):
                self.leftController.resetBackwardCollided()
                self.rightController.resetBackwardCollided()
                return True
        else:
                return False

    def move(self, movement):
        """
                movement is expected to contain the three values from the Move message
        """
        leftWheel, rightWheel, rampUp = movement

        # place the appropriate commands on each controllerQ
        synchronizer.clear()
        self.leftQ.put((leftWheel, rampUp))
        self.rightQ.put((-(rightWheel), rampUp))
        synchronizer.set()

        return

class MotionMind(threading.Thread):
    def __init__(self, name, serialPort, address, commandQueue):
        rospy.logdebug("initialzing MotionMind")
        
        threading.Thread.__init__(self)

        self.name = name
        self.commandQueue = commandQueue
        self.currentVelocity = 0
        self.forwardCollision = False
        self.forwardCollided = False
        self.backwardCollision = False
        self.backwardCollided = False
        self.intervalDelay = 1.0
        self.serialPort = serial.Serial(port = serialPort,
                                                                        baudrate = 9600,
                                                                        timeout = 1)
        self.serialPort.setRTS(level = True)
        self.serialPort.setDTR(level = True)
        self.address = address
        # set all of the PID parameters
        self.sendCommand("W%s 04 10000" % (self.address))# P gain
        rospy.logdebug(self.readResponse())
        self.sendCommand("W%s 05 0" % (self.address))    # I gain
        rospy.logdebug(self.readResponse())
        self.sendCommand("W%s 06 0" % (self.address))    # D gain
        rospy.logdebug(self.readResponse())
        self.sendCommand("W%s 08 10000" % (self.address))# PID scalar
        rospy.logdebug(self.readResponse())

        return

    def resetForwardCollided(self):
        self.forwardCollided = False

        return

    def checkForCollision(self):
        self.serialPort.flushInput()
        self.sendCommand("R%s 16" % (self.address))
        response = self.readResponse()
        rospy.logdebug(self.address)
        rospy.logdebug(response)

        if (self.name == "left"):
            forwardCollisionBit = 2
            backwardCollisionBit = 1
        else:
            forwardCollisionBit = 1
            backwardCollisionBit = 2


        if (int(response.split('=')[1]) & forwardCollisionBit):
            self.forwardCollision = True
            self.forwardCollided = True
        else:
            self.forwardCollision = False

        if (int(response.split('=')[1]) & backwardCollisionBit):
            self.backwardCollision = True
            self.backwardCollided = True
        else:
            self.backwardCollision = False

        return

    def run(self):
        while True:
            newVelocity, rampUpSeconds = self.commandQueue.get(True)

            synchronizer.wait()

            velocityChange = newVelocity - self.currentVelocity
            velocityIncrement = int(velocityChange / rampUpSeconds)
            intervals = rampUpSeconds

            while (intervals > 0):
                self.checkForCollision()
                self.currentVelocity = self.currentVelocity + velocityIncrement
                command = "V%s %d" % (self.address, self.currentVelocity)
                self.sendCommand(command)
                self.readResponse()
                intervals = intervals - 1
                if (intervals > 0):
                    time.sleep(self.intervalDelay)
                self.checkForCollision()

            if (newVelocity != self.currentVelocity):
                command = "V%s %d" % (self.address, newVelocity)
                self.sendCommand(command)
                self.readResponse()
                self.currentVelocity = newVelocity
                self.checkForCollision()

    def readResponse(self):
        response = ""
        characterRead = self.serialPort.read(1)
        while (characterRead != '\n'):
            response = response + characterRead
            characterRead = self.serialPort.read(1)
        if (response[-1] == '\r'):
            response = response[:-1]

        return response

    def sendCommand(self, command):

        self.serialPort.write("%s\r\n" % (command))

        return
