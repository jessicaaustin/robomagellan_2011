import roslib; roslib.load_manifest('robomagellan')

from robomagellan.msg import Collision

import rospy
from std_msgs.msg import String

class CollisionPublisher():
    def __init__(self):
        rospy.loginfo("initialzing CollisionPublisher")
        self.publisher = rospy.Publisher('collision', Collision)

    def publish_collision(self, forward_collision, backward_collision):
        c = Collision()
        c.forwardCollision = forward_collision
        c.backwardCollision = backward_collision
        self.publisher.publish(c)

