#!/usr/bin/env python3

import rospy
from geometry_msgs import Twist


def publisher():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('publisher', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    
    for x in range(0,3):
        msg = Twist
        msg.linear.x = 1
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep(10)
        msg.linear.x = 0
        msg.angular.z = 1
        pub.publish(msg)
        rate.sleep(10)



if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
