#!/usr/bin/env python
import json
from math import cos, sin, sqrt, atan2
from geometry_msgs.msg import Point, Pose
from tf.transformations import *
from copy import deepcopy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import patches
import numpy as np
from scipy.linalg import norm

class FallenTarget:

    # ASSUMING THESE POINTS ARE IN THE WORLDFRAME
    
    def __init__(self, point1, point2, radius): # add stuff later for plotting
        self.point1 = point1
        self.point2 = point2
        self.radius = radius

        #points where the gripper ends up
        self.var_point1 = deepcopy(point1)
        self.var_point2 = deepcopy(point2)

        self.rot_x = 0 #assuming only working with cans, therefore can essentially 'rolls' about x axis.
        self.rot_y = atan2(point2.z-point1.z, point2.x-point1.x) #subject to change -- check robot setup
        self.rot_z = atan2(point2.y-point1.y, point2.x-point1.x) #subject to change -- check robot setup

        self.cm = Point((point1.x+point2.x)/2,(point1.y+point2.y)/2,(point1.z+point2.z)/2)
        self.var_cm = deepcopy(self.cm)

        self.length = sqrt((point2.x-point1.x)**2+(point2.y-point1.y)**2+(point2.z-point1.z)**2) #we assume this does not change

    def update_cm_x(self,val):
        self.var_cm.x=val
        self.var_point1.x = self.cm.x - self.length/2 *cos(self.rot_z) *cos(self.rot_y)
        self.var_point2.x = self.cm.x + self.length/2 *cos(self.rot_z) *cos(self.rot_y)

    def update_cm_y(self,val):
        self.var_cm.y=val
        self.var_point1.y = self.cm.y - self.length/2 *sin(self.rot_z) *cos(self.rot_y)
        self.var_point2.y = self.cm.y + self.length/2 *sin(self.rot_z) *cos(self.rot_y)
    
    def update_cm_z(self,val):
        self.var_cm.z=val
        self.var_point1.z = self.cm.z - self.length/2 *sin(self.rot_y) *cos(self.rot_z)
        self.var_point2.z = self.cm.z + self.length/2 *sin(self.rot_y) *cos(self.rot_z)

    def plot(self):
        can_side_long = patches.Rectangle(((self.cm.x - self.radius), 0), self.radius*2, self.var_point.z, color = "slategray")
        can_side_circ = patches.Circle()




        

