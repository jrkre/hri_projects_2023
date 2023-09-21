#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# this file will sense an obstacle and try to avoid it

turn = 0

#find closest object angle
def findObjPos(data):
    global turn
    closest = 100
    closestAngle = -1
    print("data.ranges.length: ", len(data.ranges))
    for i in range(0, len(data.ranges)):
        if data.ranges[i] < closest:
            
            closest = data.ranges[i]
            closestAngle = i
    #print("Object detected at angle: ", closestAngle)
    #print("Object detected at distance: ", closest)
    if closestAngle < 100: # 340- is on the left side of the robot -> do nothing
        turn = 0
    elif closestAngle > 980: # 740+ is on the right side of the robot -> do nothing
        turn = 0
    elif closest < .5: #this should be right in front of it
        turn = 1
    print(turn)

def subscriber():
    rospy.Subscriber('/base_scan', LaserScan, findObjPos)

def driver():
    msg = Twist()
    
    while not rospy.is_shutdown():
        if (turn == 0):
            msg.linear.x = 1
            msg.angular.z = 0
        elif (turn != 0):
            msg.linear.x = .1
            msg.angular.z = 1
        pub.publish(msg)
        rospy.sleep(.1)


if __name__ == '__main__':
    try:
        rospy.init_node('obstacle_stoppies', anonymous=True)
        rate = rospy.Rate(1) # 10hz
        
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/base_scan', LaserScan, findObjPos)
        driver()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
