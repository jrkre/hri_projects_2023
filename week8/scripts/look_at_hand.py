#!/usr/bin/env python3
# license removed for brevity
import rospy
import tf2_ros
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool
import math

current_joint_state = None
enable = False


def find_head_angle():
    tfBuffer = tf2_ros.Buffer() #tf buffer length
    listener = tf2_ros.TransformListener(tfBuffer)
    
    trans = tfBuffer.lookup_transform('Neck', 'l_gripper', rospy.Time(), rospy.Duration(1.0))
    
    return trans

def callback(msg):
    global current_joint_state
    current_joint_state = msg
    
def enable_callback(msg):
    global enable
    enable = msg.data

def looker():
    global current_joint_state, enable
    
    rospy.init_node('looker', anonymous=True)
    
    rospy.Subscriber("/look_at_hand/enable", Bool, enable_callback)
    pub = rospy.Publisher('/animation', JointState, queue_size=10)
    
    rospy.Subscriber("joint_states", JointState, callback)
    
    while current_joint_state == None:
        rospy.loginfo("waiting for joint state")
        rospy.sleep(0.1)
    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        
        joint_states = JointState()
        joint_states.header.frame_id="base_link"
        
        joint_states.name.append("HeadYaw")
        joint_states.name.append("HeadPitch")
        
        transform = find_head_angle()
        
        head_yaw = current_joint_state.position[0]
        head_pitch = current_joint_state.position[1]
        
        yaw = math.atan2(transform.transform.translation.y, transform.transform.translation.x)
        pitch = -math.atan2(transform.transform.translation.z, transform.transform.translation.x) #- (math.pi / 2)
        
        print ("transform: " + str(transform))
        print ("yaw: " + str(yaw) + " pitch: " + str(pitch))
        
        joint_states.header.stamp = rospy.get_rostime()

        joint_states.position.append(head_yaw + yaw)
        joint_states.position.append(pitch)
        if enable:
            print("publishing")
            pub.publish(joint_states)
        print(str(enable))
        rate.sleep()


if __name__ == '__main__':
    try:
        looker()
    except rospy.ROSInterruptException:
        pass
