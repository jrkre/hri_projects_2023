#!/usr/bin/env python3


import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan


def callback(data):
    closest = 100
    for i in range(len(data.ranges)):
        if data.ranges[i] < closest:
            closest = data.ranges[i]
    print(closest)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/base_scan', LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
