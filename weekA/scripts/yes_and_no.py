#!/usr/bin/env python3

import rospy

from std_msgs.msg import String


yes_received = False

cnt = 0

response = ""

def result_callback(msg):
    global response
    response = msg.data

if __name__ == "__main__":
    # code to be executed when the script is run as the main program
    rospy.init_node("yes_and_no")
    pub = rospy.Publisher("/tts/phrase", String, queue_size=10)
    sub = rospy.Subscriber("/speech_recognition/final_result", String, result_callback)
    
    outside = False

    pub.publish("do you want to go outside?")
    rospy.loginfo("do you want to go outside?")
    
    rospy.sleep(1)
    while response != "yes":
        if response == "no":
            pub.publish("ok")
            rospy.sleep(1)
            break
        if response == "what":
            pub.publish("do you want to go outside?")
        if response == "yes":
            pub.publish("ok")
            outside = True
        rospy.sleep(1)
    
    
    pub.publish("are you sure?")
    
    while response != "yes":
        rospy.sleep(1)
    
    
    rospy.Rate(1)
    rospy.spin()