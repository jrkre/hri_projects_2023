#!/usr/bin/env python3

import rospy

from std_msgs.msg import String


def result_callback(msg):
    
    rospy.loginfo("speech received: " + msg.data)
    
    pub.publish(msg.data)


if __name__ == "__main__":
    # code to be executed when the script is run as the main program
    rospy.init_node("repeater")
    pub = rospy.Publisher("tts/phrase", String, queue_size=10)
    sub = rospy.Subscriber("/speech_recognition/final_result", String, result_callback)


    rospy.Rate(1)
    rospy.spin()