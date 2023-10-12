#!/usr/bin/env python3
import rospy

# to get commandline arguments
import sys
import numpy as np

# because of transformations
import tf

import tf2_ros
import geometry_msgs.msg
from people_msgs.msg import PositionMeasurement, PositionMeasurementArray

people = None
people_arr = None

def publish_peeps(msg):
    global people, people_arr
    broadcaster = tf2_ros.TransformBroadcaster()
    transformStamped = geometry_msgs.msg.TransformStamped()
    
    people = len(msg.people)
    
    for i in range(0, len(msg.people)):
        transformStamped.header.stamp = rospy.Time.now()
        transformStamped.header.frame_id = "odom"
        transformStamped.child_frame_id = "human" + str(i)
        transformStamped.header.stamp = rospy.get_time()
    
        if (len(msg.people) == 0):
            return
    
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
    
        #broadcaster.sendTransform(transformStamped)
    
    people_arr = msg.people
    print("people: ", people)


def classify_line(person_array):
    global people
    line = False
    transformListener = tf.TransformListener()
    
    det_arr = np.zeros(shape=(len(person_array), 3))
    
    if (len(person_array) < 3):
        return True
    
    for i in range(0, len(person_array)):
        det_arr[i] = np.array([1, person_array[i].pos.x, person_array[i].pos.y])
        print(det_arr[i])
    
    det = np.linalg.det(det_arr)
    
    print(det_arr)
    print(det)
    
    if (det < 1.5):
        line = True
        
    return line
        
    


if __name__ == '__main__':

    rospy.init_node('line_detector')
    subscriber = rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray, publish_peeps)
    
    pub = rospy.Publisher('/robot_0/detected_groups', PositionMeasurementArray, queue_size=10)
    
    while (people == None):
        print("waiting for people data")
        rospy.sleep(.1)
    while (people_arr == None):
        print("waiting for people_arr data")
        rospy.sleep(.1)
    
    
    while(rospy.is_shutdown() == False):
        print(classify_line(people_arr))
        rospy.sleep(.1)
    
    rospy.spin()

