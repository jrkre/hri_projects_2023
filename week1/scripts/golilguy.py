#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist


def publisher():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('publisher', anonymous=True)
    rate = rospy.Rate(1) # 10hz

    for x in range(0,200):
        msg = Twist()
        msg.linear.x = 10
        rospy.loginfo(msg)
        pub.publish(msg)
        rospy.sleep(.1)


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
