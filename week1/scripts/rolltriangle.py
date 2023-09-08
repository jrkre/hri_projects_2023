#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist


def publisher():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('publisher', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    
    for i in range (0,3):
        for x in range(0,20):
            msg = Twist()
            msg.linear.x = 1
            rospy.loginfo(msg)
            pub.publish(msg)
            rospy.sleep(.1)
        for x in range (0,21):
            msg.linear.x = 0
            msg.angular.z = 1
            rospy.loginfo(msg)
            pub.publish(msg)
            rospy.sleep(.1)


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
