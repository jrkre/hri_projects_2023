#!/usr/bin/env python3

import rospy

from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState

response = ""

def callback(msg):
    global response
    response = msg.data
    print(response)

if __name__ == '__main__':
    rospy.init_node('nao_stuff')
    
    speech_pub = rospy.Publisher('/tts/phrase', String, queue_size=10)
    animation_pub = rospy.Publisher('/animate', Bool, queue_size=10)
    hand_look_pub = rospy.Publisher('/look_at_hand/enable', Bool, queue_size=10)
    sub = rospy.Subscriber('/speech_recognition/final_result', String, callback)
    
    while (rospy.is_shutdown() == False):
        if response == "look at hand":
            for i in range(10):
                hand_look_pub.publish(True)
                rospy.sleep(0.1)
        elif response == "stop looking":
            for i in range(10):
                hand_look_pub.publish(False)
                rospy.sleep(0.1)
        elif response == "animate":
            animation_pub.publish(True)
        elif response == "say something":
            speech_pub.publish("hello")
        elif response == "stop":
            speech_pub.publish("okay")
        