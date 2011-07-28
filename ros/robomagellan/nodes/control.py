#!/usr/bin/env python
import roslib; roslib.load_manifest('robomagellan')
import rospy
from robomagellan.msg import Move
from robomagellan.msg import Range
from robomagellan.msg import Gyro
from robomagellan.msg import Collision

def rangeCallback(rangeMessage):

    rospy.logdebug(rospy.get_name() + ' Range: %d',
            rangeMessage.rangeInCm)

    if (rangeMessage.rangeInCm > 60):
        rospy.loginfo(rospy.get_name() + ' Moving')

        try:
             publisher.publish(moveCommand)

        except:
             rospy.loginfo('Unable to publish motor command')
    else:
        try:
            #
            # "escape" behavior
#
            rospy.loginfo('Stopping')
            publisher.publish(stopCommand)
            rospy.sleep(1.0)
            publisher.publish(backCommand)
            rospy.sleep(1.0)
            publisher.publish(rightTurnCommand)
            rospy.sleep(1.0)
            publisher.publish(stopCommand)

        except:
            rospy.loginfo('Unable to publish motor command')


def gyroCallback(gyroMessage):
    rospy.logdebug(rospy.get_name() + ' Gyro rate, direction: %d, %s',
            gyroMessage.rotationRate,
            gyroMessage.rotationDirection)

    if (gyroMessage.rotationDirection == 'L'):
        rospy.loginfo(rospy.get_name() + ' Correcting left drift')
        motorCommand = moveCommand
    else:
        rospy.loginfo(rospy.get_name() + ' Too close')
        motorCommand = rightTurnCommand

    try:
        publisher.publish(motorCommand)

    except:
        rospy.loginfo('Unable to publish motor command')

def collisionCallback(collisionMessage):
    rospy.logdebug(rospy.get_name() + ' Collision state, Forward: %s, Backward: %s',
                   collisionMessage.forwardCollision,
                   collisionMessage.backwardCollision)

    if (collisionMessage.forwardCollision):
        publisher.publish(stopCommand)
        publisher.publish(backCommand)

def controlMotors():
    rospy.init_node('control', anonymous=True)
    rospy.loginfo(rospy.get_name() + ' Started')
    rospy.Subscriber("rangeTopic", Range, rangeCallback)
    rospy.spin()

if __name__ == '__main__':
    currentState = 'STOPPED'
    rightWheelSpeed = 300
    leftWheelSpeed  = 300
    
    moveCommand = Move(leftWheel = leftWheelSpeed,
                       rightWheel = rightWheelSpeed,
                       rampUp = 1)
    stopCommand = Move(leftWheel = 0,
                       rightWheel = 0,
                       rampUp = 1)
    backCommand = Move(leftWheel = -100,
                       rightWheel = -100,
                       rampUp = 1)
    rightTurnCommand = Move(leftWheel = leftWheelSpeed,
                       rightWheel = 0,
                       rampUp = 1)
    
    publisher = rospy.Publisher('motionTopic', Move)

    controlMotors()
