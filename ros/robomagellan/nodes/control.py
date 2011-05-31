#!/usr/bin/env python
import roslib; roslib.load_manifest('robomagellan')
import rospy
from robomagellan.msg import Move
from robomagellan.msg import Range
from robomagellan.msg import Gyro

currentState = 'STOPPED'
#
rightWheelSpeed = 0
leftWheelSpeed  = 0
moving = False
turning = False

moveCommand = Move(leftWheel = leftWheelSpeed,
				   rightWheel = rightWheelSpeed)

publisher = rospy.Publisher('motionTopic', Move)

def rangeCallback(rangeMessage):

	rospy.logdebug(rospy.get_name() + ' Range: %d',
		rangeMessage.rangeInCm)

	if (rangeMessage.rangeInCm > 60):
		if (not moving and not turning):
			moving = True
			moveCommand.leftWheel = 500
			moveCommand.rightWheel = 500
			rospy.loginfo(rospy.get_name() + ' Starting')
			motorCommand = moveCommand
			publisher.publish(motorCommand)
	else:
		if (not turning):
			if (moving):
				moveCommand.leftWheel = 0
				moveCommand.rightWheel = 0
				rospy.loginfo(rospy.get_name() + ' Stopping')
				motorCommand = moveCommand
				publisher.publish(motorCommand)
				moving = False

	try:
		publisher.publish(motorCommand)

	except:
		rospy.loginfo('Unable to publish motor command')

def gyroCallback(gyroMessage):
	rospy.logdebug(rospy.get_name() + ' Gyro rate, direction: %d, %c',
		gyroMessage.rotationRate,
		gyroMessage.rotationDirection)

	if (gyroMessage.rotatonDirection == 'L'):
		rospy.loginfo(rospy.get_name() + ' Correcting left drift')
		motorCommand = moveCommand
	else:
		rospy.loginfo(rospy.get_name() + ' Too close')
		motorCommand = rightTurnCommand

	try:
		publisher.publish(motorCommand)

	except:
		rospy.loginfo('Unable to publish motor command')


		
def controlMotors():
    rospy.init_node('control', anonymous=True)
    rospy.loginfo(rospy.get_name() + ' Started')
    rospy.Subscriber("rangeTopic", Range, rangeCallback)
    rospy.Subscriber("gyroTopic", Gyro, gyroCallback)
    rospy.spin()

if __name__ == '__main__':
    controlMotors()
