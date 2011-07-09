#!/usr/bin/env python
import roslib; roslib.load_manifest('robomagellan')
import rospy
import MotionMindPair
from robomagellan.msg import Move

motors = MotionMindPair.Mover()

def callback(moveMessage):
    rospy.logdebug(rospy.get_name() + ' Left: %d, Right: %d, RampUp: %d',
            moveMessage.leftWheel,
            moveMessage.rightWheel,
            moveMessage.rampUp)

    motors.move((moveMessage.leftWheel,
                             moveMessage.rightWheel,
                             moveMessage.rampUp))

def controlMotors():
    rospy.init_node('motors', anonymous=True)
    rospy.loginfo(rospy.get_name() + ' Started')
    rospy.Subscriber("motionTopic", Move, callback)
    rospy.spin()

if __name__ == '__main__':
    controlMotors()
