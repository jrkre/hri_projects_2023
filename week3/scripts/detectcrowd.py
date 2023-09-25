#!/usr/bin/env python3

import rospy

# to get commandline arguments
import sys

# because of transformations
import tf

import tf2_ros
import geometry_msgs.msg
from people_msgs.msg import PositionMeasurement, PositionMeasurementArray

crowd = None

def findCrowd(peoplearray):
    broadcaster = tf2_ros.TransformBroadcaster()
    transformStamped = geometry_msgs.msg.TransformStamped()
    
    transformStamped.header.stamp = rospy.Time.now()
    transformStamped.header.frame_id = "odom"
    transformStamped.child_frame_id = "human_group"
    
    front_of_crowd = 0
    
    # find the center of multiple humans
    for i in range(0, len(peoplearray.people)):
        
        transformStamped.header.stamp = rospy.Time.now()
        transformStamped.header.frame_id = "odom"
        transformStamped.child_frame_id = "human_" + str(i)
    
        person_x = peoplearray.people[i].pos.x
        person_y = peoplearray.people[i].pos.y
        person_z = 0
        
        transformStamped.transform.translation.x += float(person_x)
        transformStamped.transform.translation.y += float(person_y)
        transformStamped.transform.translation.z += float(person_z)
        
        q = tf.transformations.quaternion_from_euler(0,0,0)
        transformStamped.transform.rotation.x = q[0]
        transformStamped.transform.rotation.y = q[1]
        transformStamped.transform.rotation.z = q[2]
        transformStamped.transform.rotation.w = q[3]
        
        broadcaster.sendTransform(transformStamped)
        

if __name__ == '__main__':
    rospy.init_node('my_tf2_broadcaster')
    subscriber = rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray, findCrowd)
    
    rospy.spin()