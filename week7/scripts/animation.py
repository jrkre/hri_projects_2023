#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import JointState
import tf

STEPS_PER_KEYFRAME = 100
PUBLISH_RATE = 100

current_state = JointState()
keyframe_jointstates = []
full_animation = []


#grab joint names
names = None

#shoulder pitch,roll, elbow yaw, roll, wrist yaw, hand
arm_initial_states = [1.5, 0, 1.35, 0.8, 0, 0, 1.5, 0, 1.35, 0.8, 0, 0]
pub = None


def driver():
    exit_entering = False
    while rospy.is_shutdown() == False:
        user_input = input("enter one of the following:\n   0 to finish entering states\n    1 to set a joint state\n    2 to run animation\n    3 to view/edit joint states\n$")
        if (user_input == "0"):
            rospy.loginfo("states entered:")
            rospy.loginfo(len(keyframe_jointstates))
            exit_entering = True
            rospy.loginfo("finished entering states. generating animation...")
            #linear interpolation between each pair of keyframes
            interpolate_list()
            
        elif (user_input == "1"):
            rospy.loginfo("saved current state as a keyframe")
            #add joint state to list of keyframes
            append_joint_state()
            
        elif (user_input == "2" and exit_entering):
            rospy.loginfo("running animation...")
            #break loop and publish animation
            for joint_state in full_animation:
                joint_state.header.stamp = rospy.Time.now()
                pub.publish(joint_state)
                rospy.loginfo("published joint state")
                rospy.sleep(0.01)
        elif (user_input == "3"):
            rospy.loginfo("joint states:")
            rospy.loginfo(len(keyframe_jointstates))
            editing_joint_state = True
            while(editing_joint_state):
                joint_state_select = input("enter a joint state number to view/edit it\n$")
                if (joint_state_select.isdigit() and int(joint_state_select) < len(keyframe_jointstates)):
                    rospy.loginfo("publishing joint state...")
                    pub.publish(keyframe_jointstates[int(joint_state_select)])
                    joint_state_edit = input("enter 1 to edit joint state and 0 to exit\n$")
                    if (joint_state_edit.isdigit() and int(joint_state_edit) == 1):
                        rospy.loginfo("editing joint state...")
                        joint_state = keyframe_jointstates[int(joint_state_select)]
                        keyframe_jointstates[int(joint_state_select)] = joint_state
                        joint_state_enter = input("enter 1 to enter joint state and 0 to exit\n$")
                        if (joint_state_enter.isdigit() and int(joint_state_enter) == 1):
                            joint_state = current_state
                            keyframe_jointstates[int(joint_state_select)] = joint_state
                            rospy.loginfo("joint state updated")
                        elif (joint_state_enter.isdigit() and int(joint_state_enter) == 0):
                            editing_joint_state = False
                            rospy.loginfo("exiting...")
                    elif(joint_state_edit.isdigit() and int(joint_state_edit) == 0):
                        editing_joint_state = False
                        rospy.loginfo("exiting...")
                    else:
                        rospy.loginfo("invalid input")
                
        else:
            rospy.loginfo("invalid input")
    
    
    

def add(a, b):
    return a + b

#loop the list of keyframes and interpolate between each pair linearly
def interpolate_list():
    global current_state, full_animation, keyframe_jointstates
    rospy.loginfo("interpolating between keyframes")
    slopes_matrix = [None] * (len(keyframe_jointstates) - 1)
    #rospy.loginfo("slopes matrix:")
    #rospy.loginfo(slopes_matrix)
    cnt = 1
    while (cnt < len(keyframe_jointstates)): #form the set of slopes between each pair of keyframes
        #rospy.loginfo("interpolating between keyframes cnt:" + str(cnt) + " and keyframeJS:" + str(len(keyframe_jointstates)) + " and slope:" + str(len(slopes_matrix)))
        
        slopes_matrix[cnt-1] = interpolate(keyframe_jointstates[cnt - 1], keyframe_jointstates[cnt])
        cnt += 1
    #generate the interpolations
    #rospy.loginfo(slopes_matrix)
    full_animation.append(keyframe_jointstates[0])
    cnt = 1
    for keyframe_diff in slopes_matrix:
        keyframe_diff = list(keyframe_diff)
        #rospy.loginfo("keyframe_diff:")
        #rospy.loginfo(keyframe_diff)
        for i in range(0, STEPS_PER_KEYFRAME):
            joint_state = JointState()
            #joint_state.header.stamp = rospy.Time.now()
            joint_state.header.frame_id = ""
            joint_state.name = names
            joint_state.position  = list(map(add, full_animation[cnt-1].position, keyframe_diff))
            #rospy.loginfo("joint_state.position:")
            #rospy.loginfo(joint_state.position[0])
            joint_state.velocity = []
            joint_state.effort = []
            full_animation.append(joint_state)
            cnt += 1
    rospy.loginfo("animation generated")
    #rospy.loginfo(full_animation)
    
    
def interpolate(js1, js2):
    if (len(js1.name) != len(js2.name)):
        rospy.loginfo("you done a bad!")
        return
    cnt = 0
    slopes = []
    for pos in js1.position:
        if (js1.position[cnt] == js2.position[cnt]):
            slopes.append(0)
        else:
            slopes.append((js2.position[cnt] - js1.position[cnt]) / STEPS_PER_KEYFRAME) #distance between keyframes over 20 'units'
        cnt += 1
    #rospy.loginfo("slopes:")
    #rospy.loginfo(slopes)
    return slopes
        

def append_joint_state():
    global current_state, keyframe_jointstates
    rospy.loginfo("joint state saved")
    keyframe_jointstates.append(current_state)

def init():
    global arm_initial_states, pub
    init_joint_state = JointState()
    init_joint_state.header.stamp = rospy.Time.now()
    init_joint_state.header.frame_id = ""
    #init_joint_state.name = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand", "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand"]
    init_joint_state.name = ["HeadYaw","HeadPitch","LHipYawPitch","LHipRoll","LHipPitch","LKneePitch","LAnklePitch","LAnkleRoll","RHipYawPitch",
                             "RHipRoll","RHipPitch","RKneePitch","RAnklePitch","RAnkleRoll","LShoulderPitch","LShoulderRoll","LElbowYaw","LElbowRoll","LWristYaw","LHand","RShoulderPitch","RShoulderRoll","RElbowYaw","RElbowRoll","RWristYaw","RHand","RFinger23","RFinger13","RFinger12","LFinger21","LFinger13","LFinger11","RFinger22","LFinger22","RFinger21","LFinger12","RFinger11","LFinger23","LThumb1","RThumb1","RThumb2","LThumb2"]
    
    init_joint_state.position = [0.0, -9.093359999989836e-05, -0.00010594240000005861, -0.0001550409999999669, -0.00038482599999989375, 
                                 -0.00016400378000000493, -0.6278536161, 0.23566920370000005, -0.00010594240000005861, -0.0001959274999999705, -0.00038482599999989375, -0.00016400378000000493, -4.8639999999711137e-05, -0.0001017730000000272, -0.15600811599999997, -0.00014643740000003236, 0.0, -0.7897633000000001, 0.0, 0.0, 0.5372685920000002, -1.0856085988, 2.08567, 0.60361563778, 0.33851027199999995, 0.7452, 0.7451247348, 0.7451247348, 0.7451247348, 0.0, 0.0, 0.0, 0.7451247348, 0.0, 0.7451247348, 0.0, 0.7451247348, 0.0, 0.0, 0.7451247348, 0.7451247348, 0.0]
    
    init_joint_state.velocity = []
    init_joint_state.effort = []
    
    rospy.loginfo("initializing robot to initial state")
    
    rospy.loginfo(init_joint_state)
    
    pub.publish(init_joint_state)

def joint_state_callback(msg):
    global current_state, names
    names = msg.name
    #print(msg.name)
    current_state = msg

if __name__ == "__main__":
    # code to be executed when the script is run as the main program
    rospy.init_node("animation")
    pub = rospy.Publisher("joint_states", JointState, queue_size=10)
    sub = rospy.Subscriber("/joint_states", JointState, joint_state_callback)
    init()
    driver()
    rospy.Rate(PUBLISH_RATE)
