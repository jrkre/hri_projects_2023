#!/usr/bin/env python3

import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import tf
import tf2_ros
import geometry_msgs.msg
from people_msgs.msg import PositionMeasurement, PositionMeasurementArray



roll, pitch, yaw = None, None, None
x, y = None, None
orientation = None
person_position = None
OdometryData = None
obstacle, obstacleDistance = None, None

def callback(msg):
    global person_position
    broadcaster = tf2_ros.TransformBroadcaster()
    transformStamped = geometry_msgs.msg.TransformStamped()
    
    transformStamped.header.stamp = rospy.Time.now()
    transformStamped.header.frame_id = "odom"
    transformStamped.child_frame_id = "human"
    
    if (len(msg.people) == 0):
        return
    
    #print("msg.people[0].pos: ", msg.people[0].pos)
    
    person_position = msg.people[0].pos
    
    x = msg.people[0].pos.x
    y = msg.people[0].pos.y
    person_z = 0
    
    transformStamped.transform.translation.x = float(x)
    transformStamped.transform.translation.y = float(y)
    transformStamped.transform.translation.z = float(person_z)
    
    quaterny_boi = tf.transformations.quaternion_from_euler(0,0,person_z)
    transformStamped.transform.rotation.x = quaterny_boi[0]
    transformStamped.transform.rotation.y = quaterny_boi[1]
    transformStamped.transform.rotation.z = quaterny_boi[2]
    transformStamped.transform.rotation.w = quaterny_boi[3]
    
    broadcaster.sendTransform(transformStamped)
    

def findObjPos(data):
    global obstacle, obstacleDistance
    closest = 100
    closestAngle = -1
    obstacle = 0
    #print("data.ranges.length: ", len(data.ranges))
    for i in range(0, len(data.ranges)):
        if data.ranges[i] < closest:
            
            closest = data.ranges[i]
            closestAngle = i
    #print("Object detected at angle: ", closestAngle)
    #print("Object detected at distance: ", closest)
    if closestAngle < 100: # 340- is on the left side of the robot -> do nothing
        obstacle = 0
    elif closestAngle > 980: # 740+ is on the right side of the robot -> do nothing
        obstacle = 0
    elif closest < .5: #this should be right in front of it
        obstacle = 1
    obstacleDistance = closest
    #print("closest: ", closest)
    rospy.sleep(.1)


#this was helpful https://www.theconstructsim.com/ros-qa-135-how-to-rotate-a-robot-to-a-desired-heading-using-feedback-from-odometry/
def updatePosition(data):
    global roll, pitch, yaw, OdometryData, orientation, x, y
    
    OdometryData = data
    
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    
    orientation = data.pose.pose.orientation
    orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
    #print("orientation_list: ", orientation_list)
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    #print("yaw: ", yaw)


def turnToAngle(radians):
    global yaw
    target_radians = radians
    msg = Twist()
    direction = None
    
    #find closest path
    alpha = target_radians - yaw
    beta = target_radians - yaw + 2*math.pi
    gamma = target_radians - yaw - 2*math.pi
    
    if(abs(alpha) < abs(beta) and abs(alpha) < abs(gamma)):
        direction = alpha
    elif(abs(beta) < abs(alpha) and abs(beta) < abs(gamma)):
        direction = beta
    else:
        direction = gamma
    
    while (abs(target_radians - yaw) > .1):
        msg.linear.x = 0
        msg.angular.z = direction
        pub.publish(msg)
        rospy.sleep(.1)
    print("done turning")
    return
    
            
def degreesToRadians(degrees):
    return degrees * (math.pi/180)

def turnRadians(degrees): #(definitely radians)
    global yaw
    
    target_rad = yaw + degrees
    
    msg = Twist()
    direction = None
    
    if (target_rad - 2*yaw > 0):
        direction = .2
    else:
        direction = -.2
    
    while (abs(target_rad - yaw) > .1):
        msg.linear.x = 0
        msg.angular.z = direction
        pub.publish(msg)
        rospy.sleep(.1)
    


def driveForward(meters):
    global OdometryData
    
    msg = Twist()
    meters = float(meters)
    
    for i in range(0, int(meters*10)):
        msg.linear.x = 1
        msg.angular.z = 0
        pub.publish(msg)
        rospy.sleep(.1)
    return


def findAngleToPerson():
    global person_position, yaw, orientation
    
    inc_x = person_position.x - x
    inc_y = person_position.y - y
    
    theta = math.atan2(inc_y, inc_x)
    
    print("theta: ", theta)
    
    return theta

def findDistanceToPerson():
    global person_position
    
    distance = math.sqrt((person_position.x - x)**2 + (person_position.y - y)**2)
    
    print("distancetohooman: ", distance)
    
    
    return distance


def driver():
    msg = Twist()
    
    while not rospy.is_shutdown():
        distanceToPerson = findDistanceToPerson() < 1
        if (distanceToPerson):
            msg.linear.x = 0
            msg.angular.z = 0
            pub.publish(msg)
            rospy.sleep(.1)
            print("I made it to the human!")
            return


        elif(not distanceToPerson and obstacle != 0 and obstacleDistance < .5):
            turnRadians(90)
            driveForward(.5)
        else:
            angle_to_turn = findAngleToPerson()
            turnToAngle(angle_to_turn)
            driveForward(.1)

        print("here")
    return


#adding a tag (hopefully)
if __name__ == '__main__':
    try:
        rospy.init_node('obstacle_stoppies', anonymous=True)
        rate = rospy.Rate(1) # 10hz
        print("top main")
        
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        rospy.Subscriber('/odom', Odometry, updatePosition)
        rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray, callback)
        rospy.Subscriber('/base_scan', LaserScan, findObjPos)
        
        while(OdometryData == None):
            print("waiting for odom data")
            rospy.sleep(.1)
        while(yaw == None):
            print("waiting for yaw data")
            rospy.sleep(.1)
        while(person_position == None):
            print("waiting for person position data")
            rospy.sleep(.1)
        while(obstacle == None):
            print("waiting for obstacle data")
            rospy.sleep(.1)
            
        print("HERE")
        
        driver()
        
        print("bottom main")
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
