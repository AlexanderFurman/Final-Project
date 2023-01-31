#!/usr/bin/env python
from math import floor
import rospy
from std_msgs.msg import String, UInt16

class Gripper:

    def __init__(self):
        #initialize node:
        rospy.init_node('gripper_node', anonymous=True) 

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
        self.vac_pub =  rospy.Publisher("/toggle_valves", String, queue_size=10) # 0 for off, 1 for left, 2 for both
        self.length_pub = rospy.Publisher("/toggle_dist", String, queue_size=10) # length in [mm] between 0 and 98

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
        


if __name__ == '__main__':
    #add this to main
    gripper = Gripper()
    gripper.toggle_dist(37.5)
    rospy.spin()


