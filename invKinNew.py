#!/usr/bin/env python


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
"""
def inv_kin(xp, yp, zp, joint_array):

    L = 0.548
    l_base = 0.187
    l_45 = 0.355 #length from joint 4 to centre of gripper
    length = sqrt(xp**2 + yp**2)


    h = abs(zp - l_base) #distance in z direction between first joint and 4th joint
    d = length - l_45
    x = sqrt(h**2 +d**2)/2

    alpha = acos(x/L)
    print(alpha*180/pi)

    beta = acos(d/(2*x))
    print(beta*180/pi)

    theta_1 = pi/2 - (alpha+beta)
    #theta_1 = pi/2 - alpha + beta
    theta_2 = 2*alpha
    theta_3 = -(theta_1+theta_2) + pi#/2

    theta_0 = arctan2(yp, xp)


    joint_array[0] = theta_0
    joint_array[1] = -theta_1
    joint_array[2] = -theta_2
    joint_array[3] = -theta_3
    joint_array[4] = 0  

    print joint_array

    if -pi < joint_array[0] < pi and -1.745 < joint_array[1] < 1.745 and -2*pi/3 < joint_array[2] < 2*pi/3 and -2*pi/3 < joint_array[3] < 2*pi/3 and -pi < joint_array[4] < pi:
        return joint_array
    else:
        print('No solution possible without exceeding joint limits!')
        return None

"""
def inv_kin(xp, yp, zp, joint_array):
    L = 0.4
    l_base =0.175
    l_45 = 0.22-0.04 #length from joint 4 to centre of gripper
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
    joint_array[3] = 0  
    joint_array[4] = -theta_3
    

    for item in joint_array:
      print("joint: " + str(item*180/pi))

    if -pi < joint_array[0] < pi and -1.745 < joint_array[1] < 1.745 and -2*pi/3 < joint_array[2] < 2*pi/3 and -pi < joint_array[3] < pi and -2*pi/3 < joint_array[4] < 2*pi/3:
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

def motion(self):
    print("press enter to begin motion")
    raw_input()

    #start_suction(self)
    #rospy.sleep(6)
    #stop_suction(self)

  
    xp = 0.5
    yp = 0.5
    zp = 0.2

    xp1 = 0.3
    yp1 = 0.3
    zp1 = 0.3

    paths = [[xp, yp, zp], [xp, yp, zp], [xp, yp, zp+0.3], [xp1, yp1, zp1+0.2], [xp1, yp1, zp1]] #removed +0.2 from first iteration
    for i in range(len(paths)):
      if abortStatus == True:
        joint_goal = [0,0,0,0,0]
        group.go(joint_goal, wait=True)
        return

      Inv_Kin = inv_kin(paths[i][0],paths[i][1] , paths[i][2],  joint_goal)

      if not Inv_Kin == None:
        group.go(joint_goal, wait=True)
      
      #if i == 1:
        #start_suction(self)
        #checkSuction(self)
      
      if i == 4:
        Inv_Kin = inv_kin(paths[i][0]-0.2,paths[i][1], paths[i][2]-0.23,  joint_goal)
        joint_goal[3] = joint_goal[3] + pi/2+10*pi/180

        group.go(joint_goal, wait=True)

        #stop_suction(self)



      

    joint_goal = [0,0,0,0,0]
    group.go(joint_goal, wait=True)




def all_close(goal, actual, tolerance):

  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonIntefaceTutorial(object):

  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the Panda
    ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the Panda:
    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)
    planning_frame = group.get_planning_frame()

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)




    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.pub = rospy.Publisher('/gripper', UInt16, queue_size=10)
    #self.eef_link = eef_link
    #self.group_names = group_names

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

  def go_to_pose_goal(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = 0.7
    pose_goal.position.y = 0
    pose_goal.position.z = 0.4

    z_rotation = atan2(pose_goal.position.y,pose_goal.position.x)
    print z_rotation
    # Roll and Pitch orientations are available for user to chose
    orientation_E = quaternion_from_euler(0,1.57,z_rotation)
    print orientation_E
    
    pose_goal.orientation.x = orientation_E[0]
    pose_goal.orientation.y = orientation_E[1]
    pose_goal.orientation.z = orientation_E[2]
    pose_goal.orientation.w = orientation_E[3]

    # pose_goal.orientation.x = orientation_E[0]
    # pose_goal.orientation.y = orientation_E[1]
    # pose_goal.orientation.z = orientation_E[2]
    
    
    group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    group.stop()

    z_rotation = atan2(pose_goal.position.y,pose_goal.position.x)
    print z_rotation
    # Roll and Pitch orientations are available for user to chose
    orientation_E = quaternion_from_euler(0,0,z_rotation)
    print orientation_E
    
    pose_goal.orientation.x = orientation_E[0]
    pose_goal.orientation.y = orientation_E[1]
    pose_goal.orientation.z = orientation_E[2]
    pose_goal.orientation.w = orientation_E[3]

    group.set_pose_target(pose_goal)
    plan = group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    group.stop()

    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def get_coords(self):
    import numpy as np
    import requests
    import os

    TINS_IDENTIFIER_URL = os.environ.get("TINS_IDENTIFIER_URL", "http://132.69.181.97:8080/identify-tins")
    res = requests.get(TINS_IDENTIFIER_URL)
    return res.json()['snir']
    


  def go_to_joint_state(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group
    pub = self.pub

    ## BEGIN_SUB_TUTORIAL plan_to_joint_state
    ##
    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
    ## thing we want to do is move it to a slightly better configuration.
    # We can get the joint values from the group and adjust some of the values:

    joint_goal = group.get_current_joint_values()
    print joint_goal
    joint_goal[0] = 0
    joint_goal[1] = 0
    joint_goal[2] = 0
    joint_goal[3] = 0
    joint_goal[4] = 0
    group.go(joint_goal, wait=True)

    #open_gripper(self)
    #rospy.sleep(3)
    #close_gripper(self)
    ####close_gripper(self)

    #return()
  
  



    """
    print "input the position of the centre of the bottle"
    xp = float(raw_input('xp = '))
    yp = float(raw_input('yp = '))
    zp = float(raw_input('zp = '))

    print "input the position you wish to move the bottle to"
    xp1 = float(raw_input('xp1 = '))
    yp1 = float(raw_input('yp1 = '))
    zp1 = float(raw_input('zp1 = '))

    """
    


    
    #here we begin our motion
   # motion(self)

    print("press enter to begin motion")
    raw_input()
    group.set_goal_position_tolerance(0.2)
    group.set_goal_orientation_tolerance(0.1)
    #start_suction(self)
    #rospy.sleep(6)
    #stop_suction(self)



    xp, yp, zp = get_coords()
    print(xp, yp, zp)

    xp1 = 0.5
    yp1 = -0.5
    zp1 = 0.12

    print("move?")
    raw_input()

    paths = [[xp, yp, zp+0.1], [xp, yp, zp], [xp, yp, zp+0.1], [xp1, yp1, zp1+0.1], [xp1, yp1, zp1]] #removed +0.2 from first iteration
    for i in range(len(paths)):
      if abortStatus == True:
        joint_goal = [0,0,0,0,0]
        group.go(joint_goal, wait=True)
        rospy.wait(2)
        print('waiting')
        return

      Inv_Kin = inv_kin(paths[i][0],paths[i][1] , paths[i][2],  joint_goal)

      if not Inv_Kin == None:
        group.go(joint_goal, wait=True)
      
      #if i == 1:
        #start_suction(self)
        #checkSuction(self)
      
      if i == 4:
        Inv_Kin = inv_kin(paths[i][0]-0.2,paths[i][1], paths[i][2]-0.23,  joint_goal)
        joint_goal[3] = joint_goal[3] + pi/2+10*pi/180

        group.go(joint_goal, wait=True)

        #stop_suction(self)



      

    joint_goal = [0,0,0,0,0]
    group.go(joint_goal, wait=True)
    


    
    """
    #we now move to the centre of the bottle by moving downwards 20cm

    Inv_Kin_1 = inv_kin(xp, yp, zp+0.2, joint_goal)

    if not Inv_Kin_1 == None:
        group.go(joint_goal, wait=True)

        Inv_Kin_2 = inv_kin(xp, yp, zp, joint_goal)

        if not Inv_Kin_2 == None:
            group.go(joint_goal, wait=True)
            #start_suction(self)

            Inv_Kin_3 = inv_kin(xp1, yp1, zp+0.3, joint_goal)

            if not Inv_Kin_3 == None:
                group.go(joint_goal, wait=True)

                Inv_Kin_4 = inv_kin(xp1, yp1, zp1+0.2, joint_goal)

                if not Inv_Kin_4 == None:
                    group.go(joint_goal, wait=True)
                   # stop_suction(self)

                    Inv_Kin_5 = inv_kin(xp1, yp1, zp1, joint_goal)

                    if not Inv_Kin_5 == None:
                        group.go(joint_goal, wait=True)

                        joint_goal[0] = 0
                        joint_goal[1] = 0
                        joint_goal[2] = 0
                        joint_goal[3] = 0
                        joint_goal[4] = 0
                        group.go(joint_goal, wait=True)
                        rospy.sleep(1)
"""
    #check to see if the position values are close to xp, yp, zp:
    print(group.get_current_pose())
    print(group.get_current_joint_values())

    #rospy.sleep(1)

    #print "============ Press `Enter` to move to next point ..."
    #raw_input()





    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group


    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)









def main():
  try:
    print "============ Press `Enter` to begin and launch  moveit_commander (press ctrl-d to exit) ..."
    raw_input()
    tutorial = MoveGroupPythonIntefaceTutorial()

    print "============ Press `Enter` to execute a movement using a joint state goal ..."
    raw_input()
    tutorial.go_to_joint_state()




    print "============ Python_moveit complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
    main()