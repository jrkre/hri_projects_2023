#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
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

def animate_to(msg):
    global keyframe_jointstates
    keyframe_jointstates.append(current_state)
    keyframe_jointstates.append(msg)
    interpolate_list()
    for joint_state in full_animation:
                joint_state.header.stamp = rospy.Time.now()
                print(joint_state)
                pub.publish(joint_state)
                #rospy.loginfo("published joint state")
                rospy.sleep(0.01)
    
def animate(msg):
    global keyframe_jointstates
    
    keyframe_jointstates.append(current_state)
    new_js = JointState([0.0, -9.093359999989836e-05, -0.00010594240000005861, -0.0001550409999999669, -0.00038482599999989375, -0.00016400378000000493, -0.6278536161, 0.23566920370000005, -0.00010594240000005861, -0.0001959274999999705, -0.00038482599999989375, -0.00016400378000000493, -4.8639999999711137e-05, -0.0001017730000000272, -0.15600811599999997, -0.00014643740000003236, 0.0, -0.7897633000000001, 0.0, 0.0, 0.5372685920000002, -1.0856085988, 2.08567, 0.60361563778, 0.33851027199999995, 0.7452, 0.7451247348, 0.7451247348, 0.7451247348, 0.0, 0.0, 0.0, 0.7451247348, 0.0, 0.7451247348, 0.0, 0.7451247348, 0.0, 0.0, 0.7451247348, 0.7451247348, 0.0])
    new_js_2 = [-1.2634988859999998, -0.5014053912, -0.4105012831999999, 0.06231735199999999, -1.148276947, 0.07524282040000001, 0.35951620139999996, 0.5524426432, -0.4105012831999999, -0.18995289650000002, -1.262803576, 1.21759006039, -0.5285659869999999, -0.5334245693, -1.196757446, 0.9441881029999999, 0.7600181479999999, -1.1185788785200002, -0.061646806000000165, 0.9444, -0.683682626, -0.4570912908999999, -1.768231026, 0.4956711296800001, 0.19187112399999973, 0.2071, 0.2070790829, 0.2070790829, 0.2070790829, 0.9443046156, 0.9443046156, 0.9443046156, 0.2070790829, 0.9443046156, 0.2070790829, 0.9443046156, 0.2070790829, 0.9443046156, 0.9443046156, 0.2070790829, 0.2070790829, 0.9443046156]
    new_new_js = [0.16101372400000002, 0.48674427119999997, -1.1356713591999998, -0.3634074385, -1.363191115, 0.75588862813, -0.26648682300000004, 0.0879582739, -1.1356713591999998, -0.504771641, -1.450853473, 1.7774085592000002, 0.00228149660000021, -0.3309929238, 1.9371702960000001, -0.25509707600000003, 0.9756764260000002, -1.18666695286, 1.6644637620000002, 0.2763, 1.1024851619999998, 0.15305119620000007, -0.617775454, 0.5500208120800001, -0.1674312659999999, 0.6497, 0.6496343803000001, 0.6496343803000001, 0.6496343803000001, 0.2762720937, 0.2762720937, 0.2762720937, 0.6496343803000001, 0.2762720937, 0.6496343803000001, 0.2762720937, 0.6496343803000001, 0.2762720937, 0.2762720937, 0.6496343803000001, 0.6496343803000001, 0.2762720937]
    
    keyframe_jointstates.append(new_js)
    keyframe_jointstates.append(new_js_2)
    keyframe_jointstates.append(new_new_js)
    print("animate")
    
    interpolate_list()
    for joint_state in full_animation:
                joint_state.header.stamp = rospy.Time.now()
                
                print(joint_state)
                pub.publish(joint_state)
                #rospy.loginfo("published joint state")
                rospy.sleep(0.01)


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
            rospy.loginfo("close joint_state_publisher to run animation!")
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
    #print("a: " + str(a) + " b: " + str(b))
    return a + b

#loop the list of keyframes and interpolate between each pair linearly
def interpolate_list():
    global current_state, full_animation, keyframe_jointstates
    rospy.loginfo("interpolating between keyframes")
    print(len(keyframe_jointstates))
    slopes_matrix = [None] * (len(keyframe_jointstates) - 1)
    print(slopes_matrix)
    #rospy.loginfo("slopes matrix:")
    #rospy.loginfo(slopes_matrix)
    cnt = 0
    while (cnt < len(keyframe_jointstates) - 1): #form the set of slopes between each pair of keyframes
        #rospy.loginfo("interpolating between keyframes cnt:" + str(cnt) + " and keyframeJS:" + str(len(keyframe_jointstates)) + " and slope:" + str(len(slopes_matrix)))
        print("interpolating between keyframes cnt:" + str(cnt))
        print("keyframe:", type(keyframe_jointstates[cnt-1]))
        slopes_matrix[cnt] = interpolate(keyframe_jointstates[cnt], keyframe_jointstates[cnt + 1])
        print("interpolate", slopes_matrix[cnt])
        cnt += 1
    #generate the interpolations
    #rospy.loginfo(slopes_matrix)
    full_animation.append(keyframe_jointstates[0])
    cnt = 0
    print("slopes_matrix:")
    print(slopes_matrix)
    for keyframe_diff in slopes_matrix:
        print("keyframe_diff:")
        print(str(keyframe_diff))
        keyframe_diff = list(keyframe_diff)
        #rospy.loginfo("keyframe_diff:")
        #rospy.loginfo(keyframe_diff)
        for i in range(0, STEPS_PER_KEYFRAME):
            joint_state = JointState()
            #joint_state.header.stamp = rospy.Time.now()
            joint_state.header.frame_id = ""
            joint_state.name = names
            joint_state.position  = list(map(add, full_animation[cnt].position, keyframe_diff))
            #rospy.loginfo("joint_state.position:")
            #rospy.loginfo(joint_state.position[0])
            joint_state.velocity = []
            joint_state.effort = []
            full_animation.append(joint_state)
            cnt += 1
    rospy.loginfo("animation generated")
    #rospy.loginfo(full_animation)
    
    
def interpolate(js1, js2):
    # if (len(js1.name) != len(js2.name)):
    #     rospy.loginfo("you done a bad!")
    #     return
    cnt = 0
    slopes = []
    if len(js1.position) <= len(js2.position):
        for pos in js1.position:
            if (js1.position[cnt] == js2.position[cnt]):
                slopes.append(0)
            else:
                slopes.append((js2.position[cnt] - js1.position[cnt]) / STEPS_PER_KEYFRAME) #distance between keyframes over 20 'units'
            cnt += 1
    else:
        for pos in js2.position:
            if (js1.position[cnt] == js2.position[cnt]):
                slopes.append(0)
            else:
                slopes.append((js2.position[cnt] - js1.position[cnt]) / STEPS_PER_KEYFRAME)
    #rospy.loginfo("slopes:")
    #rospy.loginfo(slopes)
    return slopes
        

def append_joint_state():
    global current_state, keyframe_jointstates
    rospy.loginfo("joint state saved")
    rospy.loginfo(current_state)
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
    keyframe_jointstates.insert(0, msg)
    current_state = msg

if __name__ == "__main__":
    # code to be executed when the script is run as the main program
    rospy.init_node("animation")
    pub = rospy.Publisher("animation", JointState, queue_size=10)
    sub = rospy.Subscriber("animate", Bool, animate)
    sub = rospy.Subscriber("/joint_states", JointState, joint_state_callback)
    #sub = rospy.Subscriber("animate_to", JointState, animate_to)
    init()
    driver()
    rospy.Rate(PUBLISH_RATE)
