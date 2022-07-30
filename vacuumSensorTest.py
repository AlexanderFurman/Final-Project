#!/usr/bin/env python


import sys
import copy

from click import Abort
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import atan2, pi
from math import acos
from math import atan
from math import sqrt
from numpy import arctan2
from numpy import arccos
from std_msgs.msg import String
from std_msgs.msg import UInt16
from std_msgs.msg import Float64
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler
## END_SUB_TUTORIAL

#define some global variables which we will be using 
abortStatus = False
type(abortStatus)
pressure = 0
type(pressure)
suction = 'off'
type(suction)

def inv_kin(xp, yp, zp, joint_array):
    L = 0.2
    l_base =0.171
    l_45 = 0.1 #length from joint 4 to centre of gripper
    length = sqrt(xp**2 + yp**2)
    h = (zp - l_base) #distance in z direction between first joint and 4th joint
    d=l_45+h
    R = sqrt(length**2 + d**2)
    
    print("alpha = arccos(" + str(R/(2*L)) + ")")
    alpha=acos(R/(2*L)) #alpha = (pi-theta_2)/2s
    beta=atan(d/L) #beta = atan((l_45+h)/length)
    theta_1 = pi/2 - alpha - beta
    theta_2=2*alpha     #theta_2 = pi - arccos(abs(1-R**2/(2*L**2)))
    theta_3 = pi - theta_1 - theta_2

    theta_0 = arctan2(yp, xp)


    joint_array[0] = theta_0
    joint_array[1] = -theta_1
    joint_array[2] = -theta_2
    joint_array[3] = -theta_3
    joint_array[4] = 0  

    for item in joint_array:
      print("joint: " + str(item*180/pi))

    if -pi < joint_array[0] < pi and -1.745 < joint_array[1] < 1.745 and -2*pi/3 < joint_array[2] < 2*pi/3 and -2*pi/3 < joint_array[3] < 2*pi/3 and -pi < joint_array[4] < pi:
        return joint_array
    else:
        print('No solution possible without exceeding joint limits!')
        return None

def sensorCallback(data):
    global pressure
    pressure = data.data
    

def listener():

    rospy.init_node('vacuumListener', anonymous=True)

    rospy.Subscriber("/vacuumSensor", Float64, sensorCallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def checkSuction(self):
    #method to see if the suction is strong enough to lift an object
    global pressure
    global suction
    if pressure > 300:
      print("Required level of suction not reached...")
      print("Aborting suction attempt")
      stop_suction()
      suction = 'off'
      abortStatus = True

    return


def start_suction(self, goal = -0.12):

    """
    suction on
    """

    #Check if the optimal pressure has been reached: if it has we will begin suction. If not we will cancel the movement


    global suction
    global pressure

    pub = rospy.Publisher('/vacuum', UInt16, queue_size=10)
    vacuumOn = 2
    group = moveit_commander.MoveGroupCommander('gripper')
    joint_goal = group.get_current_joint_values()
    print("Current gripper joint position: " + str(joint_goal))
    if suction == 'off':
        rospy.sleep(0.5)
        pub.publish(vacuumOn)
        joint_goal[0] = goal
        group.go(joint_goal, wait=True)
        print("Vacuum turned on")
        rospy.sleep(1)
        group = moveit_commander.MoveGroupCommander('arm')
        group.stop()
        suction = 'on'
    else:
        print("SUCTION ALREADY ON")

def stop_suction(self, goal = 0):

    """
    suction off
    """
    global suction
    pub = rospy.Publisher('/vacuum', UInt16, queue_size=10)
    vacuumOn = 1
    group = moveit_commander.MoveGroupCommander('gripper')
    joint_goal = group.get_current_joint_values()
    print("Current gripper joint position: " + str(joint_goal))
    if suction == 'on':
        rospy.sleep(2)
        pub.publish(vacuumOn)
        joint_goal[0] = goal
        group.go(joint_goal, wait=True)
        print("Vacuum turned on")
        rospy.sleep(1)
        group = moveit_commander.MoveGroupCommander('arm')
        group.stop()
        suction = 'off'
    else:
        print("SUCTION ALREADY OFF")