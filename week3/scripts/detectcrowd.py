#!/usr/bin/env python3

import rospy

# to get commandline arguments
import sys

# because of transformations
import tf

import tf2_ros
import geometry_msgs.msg
from people_msgs.msg import PositionMeasurement, PositionMeasurementArray

x_min, x_max, y_min, y_max, x_avg, y_avg, is_group = 0, 0, 0, 0, 0, 0, False

crowd = None

def findAvg(peoplearray):
    global x_min, x_max, y_min, y_max, x_avg, y_avg, is_group
    is_group = False
    
    for i in range(0, len(peoplearray.people)):
        person_x = peoplearray.people[i].pos.x
        person_y = peoplearray.people[i].pos.y
        
        if person_x < x_min:
            x_min = person_x
        if person_x > x_max:
            x_max = person_x
        if person_y < y_min:
            y_min = person_y
        if person_y > y_max:
            y_max = person_y
            
        x_avg += person_x
        y_avg += person_y
        
    x_avg = x_avg / len(peoplearray.people)
    y_avg = y_avg / len(peoplearray.people)
    
    if (x_max - x_min) < 4 and (y_max - y_min) < 4:
        is_group = True
        print ("group detected @: " + str(x_avg) + ", " + str(y_avg))

def findCrowd(peoplearray):
    broadcaster = tf2_ros.TransformBroadcaster()
    transformStamped = geometry_msgs.msg.TransformStamped()
    
    transformStamped.header.stamp = rospy.Time.now()
    transformStamped.header.frame_id = "odom"
    transformStamped.child_frame_id = "human_group"
    
    findAvg(peoplearray)
    
    # find the center of multiple humans
    for i in range(0, len(peoplearray.people)):
        
        transformStamped.header.stamp = rospy.Time.now()
        transformStamped.header.frame_id = "robot_0/odom"
        transformStamped.child_frame_id = "human_" + str(i)
    
        person_x = peoplearray.people[i].pos.x
        person_y = peoplearray.people[i].pos.y
        person_z = 0
        
        transformStamped.transform.translation.x = float(person_x)
        transformStamped.transform.translation.y = float(person_y)
        transformStamped.transform.translation.z = float(person_z)
        
        q = tf.transformations.quaternion_from_euler(person_x, person_y, person_z)
        transformStamped.transform.rotation.x = q[0]
        transformStamped.transform.rotation.y = q[1]
        transformStamped.transform.rotation.z = q[2]
        transformStamped.transform.rotation.w = q[3]
        
        broadcaster.sendTransform(transformStamped)
    
    if is_group:
        transformStamped.header.stamp = rospy.Time.now()
        transformStamped.header.frame_id = "robot_0/odom"
        transformStamped.child_frame_id = "human_group"
        
        group_x = x_avg
        group_y = y_avg
        group_z = 0
        
        transformStamped.transform.translation.x = float(group_x)
        transformStamped.transform.translation.y = float(group_y)
        transformStamped.transform.translation.z = float(group_z)
        
        q = tf.transformations.quaternion_from_euler(group_x, group_y, group_z)
        transformStamped.transform.rotation.x = q[0]
        transformStamped.transform.rotation.y = q[1]
        transformStamped.transform.rotation.z = q[2]
        transformStamped.transform.rotation.w = q[3]
        
        broadcaster.sendTransform(transformStamped)
    


if __name__ == '__main__':
    rospy.init_node('my_tf2_broadcaster')
    subscriber = rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray, findCrowd)
    
    rospy.spin()