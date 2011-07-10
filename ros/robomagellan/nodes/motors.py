#!/usr/bin/env python
import roslib; roslib.load_manifest('robomagellan')
import rospy
import MotionMindPair
from robomagellan.msg import Move, Collision

motors = MotionMindPair.Mover()

def checkForCollision():
    collisionState.forwardCollision = motors.checkForwardCollision()
    collisionState.backwardCollision = motors.checkBackwardCollision()
    
    if (collisionState.forwardCollision or collisionState.backwardCollision):
        publisher.publish(collisionState)
    
    return

def callback(moveMessage):
    rospy.logdebug(rospy.get_name() + ' Left: %d, Right: %d, RampUp: %d',
            moveMessage.leftWheel,
            moveMessage.rightWheel,
            moveMessage.rampUp)

    checkForCollision()
    motors.move((moveMessage.leftWheel,
    			 moveMessage.rightWheel,
    			 moveMessage.rampUp))
    checkForCollision()


def controlMotors():
    rospy.init_node('motors', anonymous=True)
    rospy.loginfo(rospy.get_name() + ' Started')
    rospy.Subscriber("motionTopic", Move, callback)

    rospy.spin()

if __name__ == '__main__':
    publisher = rospy.Publisher('collisionTopic', Collision)
    collisionState = Collision()
    controlMotors()
