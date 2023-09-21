#!/usr/bin/env python3
import rospy

# to get commandline arguments
import sys

# because of transformations
import tf

import tf2_ros
import geometry_msgs.msg
from people_msgs.msg import PositionMeasurement, PositionMeasurementArray

def callback(msg):
    broadcaster = tf2_ros.TransformBroadcaster()
    transformStamped = geometry_msgs.msg.TransformStamped()
    
    transformStamped.header.stamp = rospy.Time.now()
    transformStamped.header.frame_id = "odom"
    transformStamped.child_frame_id = "human"

    
    person_x = msg.people[0].pos.x
    person_y = msg.people[0].pos.y
    person_z = 0
    
    transformStamped.transform.translation.x = float(person_x)
    transformStamped.transform.translation.y = float(person_y)
    transformStamped.transform.translation.z = float(person_z)
    
    quaterny_boi = tf.transformations.quaternion_from_euler(0,0,0)
    transformStamped.transform.rotation.x = quaterny_boi[0]
    transformStamped.transform.rotation.y = quaterny_boi[1]
    transformStamped.transform.rotation.z = quaterny_boi[2]
    transformStamped.transform.rotation.w = quaterny_boi[3]
    
    broadcaster.sendTransform(transformStamped)


if __name__ == '__main__':

    rospy.init_node('my_tf2_broadcaster')
    subscriber = rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray, callback)
    
    rospy.spin()