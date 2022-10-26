#!/usr/bin/env python
from math import floor
import rospy
from std_msgs.msg import String, UInt16

import os
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
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler

class Gripper:

    def __init__(self):
        # initialize sensor attributes
        self.potL = 0
        self.potR = 0
        self.vacSensor = 0

        #initialize motor + suction attributes
        self.dist = 0
        self.suction = 0 # {0,1,2} <--> {off, 1 finger actuated, both fingers actuated}

        #initialize subscribers
        self.potL_sub = rospy.Subscriber("/potentiometerLeft", UInt16, self.callback_pL)
        self.potL_sub = rospy.Subscriber("/potentiometerRight", UInt16, self.callback_pR)
        self.vac_sub = rospy.Subscriber("/vacuumSensor", UInt16, self.callback_vac)

        #initialize publishers
        self.vac_pub =  rospy.Publisher("/toggle_valves", UInt16, queue_size=10) # 0 for off, 1 for left, 2 for both
        self.length_pub = rospy.Publisher("/toggle_dist", UInt16, queue_size=10) # length in [mm] between 0 and 98

    def callback_pL(self, msg):
        self.potL = msg.data

    def callback_pR(self, msg):
        self.potR = msg.data

    def callback_vac(self, msg):
        self.vacSensor = msg.data

    def start_suction(self, setting): #1 for single finger, 2 for both
        if setting == 1 or setting == 2:
            self.suction = setting
            msg = UInt16()
            msg.data = self.suction
            self.vac_pub.publish(msg)
        else:
            print("An invalid setting was passed to the function start_suction")

    def stop_suction(self):
        self.suction = 0
        msg = UInt16()
        msg.data = self.suction
        self.vac_pub.publish(msg)

    def toggle_dist(self, length): #can input length between 0 and 98[mm]
        length = floor(length)
        min = 0
        max = 98

        if self.dist == length:
            print("Distance between fingers already ", length, "[mm].")
            return
        elif (length > max) or (length < min):
            print("Requested distance is out of bounds.")
            return
        
        self.dist = length
        msg = UInt16()
        msg.data = self.dist
        self.length_pub.publish(msg)




class MoveGroupPythonIntefaceTutorial(object):

  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    #rospy.init_node('move_group_python_interface_tutorial',anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)
  
  def go_to_joint_state(self):
    
    print ("Hello")
    
    #add this to main
    rospy.init_node('gripper_node', anonymous=True) 
    gripper = Gripper()
    rospy.sleep(1)
    # go = 'y'
    # while go == 'y':
    print("input gripper length: ")
    distance = int(input())
    gripper.toggle_dist(distance)
    rospy.sleep(5)
    gripper.toggle_dist(0)
    #     print("set another length? (y/n)")
    #     go = String(input())
    group = [0,0,0,0,0]
    
    group.go(group, wait=True)
    group.stop()
    
    #inv_kin1=inv_kin(0,-0.45,0.7,joint_goal)
    # group.go(inv_kin1, wait=True)

    print("press enter to start movement")
    # gripper.start_suction(1)
    # rospy.sleep(5)
    # gripper.stop_suction()
    # rospy.sleep(5)

    group.go([-1.57, 0,0,0,0], wait=True)
    group.stop()

    # group.go([-1.57, 0.66, 1.21, 0.00, 1.39], wait=True)
    # # Calling ``stop()`` ensures that there is no residual movement
    # group.stop()

    # gripper.start_suction(2)

    # group.go([-1.57, 0.24, 1.00, 0.00, 1.55], wait=True)
    # # Calling ``stop()`` ensures that there is no residual movement
    # group.stop()


    # group.go([-1.57, 1.01, 0.59, 0.00, 1.64], wait=True)
    # # Calling ``stop()`` ensures that there is no residual movement
    # group.stop()

    # gripper.stop_suction()

def sel_callback(ardu_msg):
    print("----------------------------------> "+ardu_msg.data+" arduino is in sync")
    global lastsync_reset 
    lastsync_reset = rospy.Time.now()
    print("----> time stamp: "+lastsync_reset.__str__())

def no_blink():
    pub_sel=rospy.Publisher('Toggle',Int8,queue_size=10)
    sub_chat=rospy.Subscriber('chatter',String,sel_callback)
    
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        print('do you want to toggle Pin? \n----1 for yes, \n----0 for no')
        val=input()
        if val==1:
            print("************yes we do************")
            pub_sel.publish(val)  
        else:
            print("************no we don't************")
        
        if lastsync_reset + rospy.Duration(secs=5) < rospy.Time.now():
            print("----------------------------------> arduini is not in sync")
        rate.sleep()
    








def main():
  try:
    tutorial = MoveGroupPythonIntefaceTutorial()
    print ("============ Press `Enter` to execute a movement using a joint state goal ...")
    '''
    global delta
    delta = rospy.Time.now()
    rospy.sleep(3)
    delta = rospy.Time.now()-delta
    '''
    tutorial.go_to_joint_state()
    #no_blink()


    print ("============ Python_moveit complete!")
    
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
    main()




# #!/usr/bin/env python


# from importlib import import_module
# from logging import shutdown
# import time
# import sys
# import copy
# import rospy
# import moveit_commander
# import moveit_msgs.msg
# import geometry_msgs.msg
# from math import atan2, pi
# from math import acos
# from math import atan
# from math import sqrt
# from numpy import arctan2
# from std_msgs.msg import String
# from std_msgs.msg import Int8
# from moveit_commander.conversions import pose_to_list
# from tf.transformations import quaternion_from_euler

# from alex_gripper import Gripper

# rospy.init_node('talker',anonymous=True)
# lastsync_reset = rospy.Time.now()
# delta = rospy.Time(secs=3)
# ## END_SUB_TUTORIAL

# def inv_kin(xp, yp, zp, joint_array):

#     L = 0.548
#     l_base = 0.159
#     l_45 = 0.28 #length from joint 4 to centre of gripper
#     length = sqrt(xp**2 + yp**2)

#     h = abs(zp - l_base) #distance in z direction between first joint and 4th joint
#     d = length - l_45
#     x = sqrt(h**2 +d**2)/2

#     alpha = acos(x/L)
#     print(alpha*180/pi)

#     beta = acos(d/(2*x))
#     print(beta*180/pi)

#     theta_1 = pi/2 - (alpha+beta)
#     #theta_1 = pi/2 - alpha + beta
#     theta_2 = 2*alpha
#     theta_3 = -(theta_1+theta_2) + pi/2
#     theta_0 = arctan2(yp, xp)

#     joint_array[0] = theta_0
#     joint_array[1] = -theta_1
#     joint_array[2] = -theta_2
#     joint_array[3] = -theta_3
#     joint_array[4] = 0

#     print(joint_array)

#     if -pi < joint_array[0] < pi and -1.745 < joint_array[1] < 1.745 and -2*pi/3 < joint_array[2] < 2*pi/3 and -2*pi/3 < joint_array[3] < 2*pi/3 and -pi < joint_array[4] < pi:
#         return joint_array
#     else:
#         print('No solution possible without exceeding joint limits!')
#         return None

# def all_close(goal, actual, tolerance):

#   all_equal = True
#   if type(goal) is list:
#     for index in range(len(goal)):
#       if abs(actual[index] - goal[index]) > tolerance:
#         return False

#   elif type(goal) is geometry_msgs.msg.PoseStamped:
#     return all_close(goal.pose, actual.pose, tolerance)

#   elif type(goal) is geometry_msgs.msg.Pose:
#     return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

#   return True

# def go_stright(group):
#   joint_goal = group.get_current_joint_values()
#   joint_goal[0] = 0
#   joint_goal[1] = 0
#   joint_goal[2] = 0
#   group.go(joint_goal, wait=True)
#   rospy.sleep(1)
#   return joint_goal

# def go_home(group):
#   joint_goal = group.get_current_joint_values()
#   joint_goal[0] = 0
#   joint_goal[1] = -1.74
#   joint_goal[2] = 2.07
#   group.go(joint_goal, wait=True)
#   rospy.sleep(1)
#   return joint_goal

# def connect_gripper(group):
  
#   starting_x=0
#   starting_y=-0.84
#   starting_z=0.2165

#   slip_in_x=starting_x
#   slip_in_y=-starting_y
#   slip_in_z=0.1165

#   exit_x=0
#   exit_y=-0.4
#   exit_z=slip_in_z
  
#   joint_goal = group.get_current_joint_values()
#   Inv_Kin_1 = inv_kin(starting_x, starting_y, starting_z, joint_goal)
#   joint_goal[3] = 0.96
#   joint_goal[4] = -0.15
#   if not Inv_Kin_1 == None:
#     group.go(joint_goal, wait=True)
#   rospy.sleep(1)
  
#   joint_goal = group.get_current_joint_values()
#   Inv_Kin_1 = inv_kin(slip_in_x, slip_in_y, slip_in_z, joint_goal)
#   joint_goal[3] = 0.96
#   joint_goal[4] = -0.15
#   if not Inv_Kin_1 == None:
#     group.go(joint_goal, wait=True)
#   rospy.sleep(1)
  
#   joint_goal = group.get_current_joint_values()
#   Inv_Kin_1 = inv_kin(exit_x, exit_y, exit_z, joint_goal)
#   joint_goal[3] = 0.96
#   joint_goal[4] = -0.15
#   if not Inv_Kin_1 == None:
#     group.go(joint_goal, wait=True)
#   rospy.sleep(1)

#   return joint_goal

# def return_gripper(group):
  
#   starting_x=0
#   starting_y=-0.84
#   starting_z=0.2165

#   slip_in_x=starting_x
#   slip_in_y=-starting_y
#   slip_in_z=0.1165
  
#   exit_x=0
#   exit_y=-0.4
#   exit_z=slip_in_z
  
#   joint_goal = group.get_current_joint_values()
#   Inv_Kin_1 = inv_kin(exit_x, exit_y, exit_z, joint_goal)
#   joint_goal[3] = 0.96
#   joint_goal[4] = -0.15
#   if not Inv_Kin_1 == None:
#     group.go(joint_goal, wait=True)
#   rospy.sleep(1)
  
#   joint_goal = group.get_current_joint_values()
#   Inv_Kin_1 = inv_kin(slip_in_x, slip_in_y, slip_in_z, joint_goal)
#   joint_goal[3] = 0.96
#   joint_goal[4] = -0.15
#   if not Inv_Kin_1 == None:
#     group.go(joint_goal, wait=True)
#   rospy.sleep(1)
  
#   joint_goal = group.get_current_joint_values()
#   Inv_Kin_1 = inv_kin(starting_x, starting_y, starting_z, joint_goal)
#   joint_goal[3] = 0.96
#   joint_goal[4] = -0.15
#   if not Inv_Kin_1 == None:
#     group.go(joint_goal, wait=True)
#   rospy.sleep(1)

#   #return joint_goal
#   return go_home(group)

# def waver(group):
#   joint_goal = group.get_current_joint_values()
#   joint_goal[4] = -1.2
#   group.go(joint_goal, wait=True)
#   rospy.sleep(8)
#   joint_goal[4] = 1.2
#   group.go(joint_goal, wait=True)
#   rospy.sleep(8)
#   return joint_goal


# class MoveGroupPythonIntefaceTutorial(object):

#   def __init__(self):
#     super(MoveGroupPythonIntefaceTutorial, self).__init__()
#     moveit_commander.roscpp_initialize(sys.argv)
#     #rospy.init_node('move_group_python_interface_tutorial',anonymous=True)
#     robot = moveit_commander.RobotCommander()
#     scene = moveit_commander.PlanningSceneInterface()
#     group_name = "arm"
#     group = moveit_commander.MoveGroupCommander(group_name)
  
#   def go_to_joint_state(self):

   
    # print ("Hello")
    
    # #add this to main
    # # rospy.init_node('gripper_node', anonymous=True) 
    # gripper = Gripper()
    # rospy.sleep(1)
    # # go = 'y'
    # # while go == 'y':
    # print("input gripper length: ")
    # distance = int(input())
    # gripper.toggle_dist(distance)
    # #     print("set another length? (y/n)")
    # #     go = String(input())
    # group = [0,0,0,0,0]

    
    

    # joint_goal=go_stright(group)
    # joint_goal = group.get_current_joint_values()

    # group.go(group, wait=True)
    # group.stop()
    
    # #inv_kin1=inv_kin(0,-0.45,0.7,joint_goal)
    # # group.go(inv_kin1, wait=True)

    # print("press enter to start movement")
    # # gripper.start_suction(1)
    # # rospy.sleep(5)
    # # gripper.stop_suction()
    # # rospy.sleep(5)


    # group.go([-1.57, 0.66, 1.21, 0.00, 1.39], wait=True)
    # # Calling ``stop()`` ensures that there is no residual movement
    # group.stop()

    # gripper.start_suction(2)

    # group.go([-1.57, 0.24, 1.00, 0.00, 1.55], wait=True)
    # # Calling ``stop()`` ensures that there is no residual movement
    # group.stop()


    # group.go([-1.57, 1.01, 0.59, 0.00, 1.64], wait=True)
    # # Calling ``stop()`` ensures that there is no residual movement
    # group.stop()

    # gripper.stop_suction()



#     ## END_SUB_TUTORIAL
#     current_joints = group.get_current_joint_values()
#     return all_close(joint_goal, current_joints, 0.01)

# def sel_callback(ardu_msg):
#     print("----------------------------------> "+ardu_msg.data+" arduino is in sync")
#     global lastsync_reset 
#     lastsync_reset = rospy.Time.now()
#     print("----> time stamp: "+lastsync_reset.__str__())

# def no_blink():
#     pub_sel=rospy.Publisher('Toggle',Int8,queue_size=10)
#     sub_chat=rospy.Subscriber('chatter',String,sel_callback)
    
#     rate = rospy.Rate(1)
#     while not rospy.is_shutdown():
#         print('do you want to toggle Pin? \n----1 for yes, \n----0 for no')
#         val=input()
#         if val==1:
#             print("************yes we do************")
#             pub_sel.publish(val)  
#         else:
#             print("************no we don't************")
        
#         if lastsync_reset + rospy.Duration(secs=5) < rospy.Time.now():
#             print("----------------------------------> arduini is not in sync")
#         rate.sleep()
    







        
# def main():
#     #add this to main
#     rospy.init_node('gripper_node', anonymous=True) 
#     gripper = Gripper()
#     # go = 'y'
#     # while go == 'y':
#     #     print("input gripper length: ")
#     #     distance = int(input())
#     #     gripper.toggle_dist(distance)
#     #     print("set another length? (y/n)")
#     #     go = String(input())
#     rospy.sleep(1)
#     gripper.start_suction(1)
#     rospy.sleep(5)
#     gripper.stop_suction()
#     rospy.sleep(5)
#     gripper.start_suction(2)
#     rospy.sleep(5)
#     gripper.stop_suction()

#     joint_goal = [0,0,0,0,0]
#     group.go(joint_goal, wait=True)

#     rospy.spin()

# if __name__ == '__main__':
#   main()