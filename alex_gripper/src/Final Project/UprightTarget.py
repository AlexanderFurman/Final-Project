#!/usr/bin/env python
import json
from math import sqrt, atan2
from geometry_msgs.msg import Point, Pose
from tf.transformations import *
from copy import deepcopy
import matplotlib.pyplot as plt
from matplotlib import patches
import numpy as np

class UprightTarget:
    
    def __init__(self, point, cm, radius):

        # ASSUMING THESE POINTS ARE IN THE WORLDFRAME
        self.init_point = point #point should be a geometry_msgs/Point(x,y,z)
        self.var_point = deepcopy(self.init_point)

        #the following will be used for visualizations
        self.cm = cm # this is a geometry_msgs/Point(x, y, z)
        self.radius = radius

    def update_point_x(self, val): #most likely not needed
        self.var_point.x = val

    def update_point_y(self, val): #most likely not needed
        self.var_point.y = val

    def update_point_z(self, val):
        self.var_point.z = val

    def plot(self):

        can_top = patches.Circle((self.cm.x, self.cm.y), self.radius, color = "slategray")

        cup = patches.Circle((self.var_point.x, self.var_point.y), self.radius/10, color = "b") # change to radius of suction cup

        unit = ' [mm]'


        can_side = patches.Rectangle(((self.cm.x - self.radius), 0), self.radius*2, self.var_point.z, color = "slategray")
        fig, ax = plt.subplots(ncols=2, sharex=True, sharey=True)

        init_to_var_x = [self.init_point.x, self.var_point.x]
        init_to_var_y = [self.init_point.y, self.var_point.y]
        ax[0].add_patch(can_top)
        ax[0].add_patch(cup)
        ax[0].plot(self.cm.x, self.cm.y, marker = "P", color = "c", markersize = 15, label = "CM")
        ax[0].plot(self.init_point.x, self.init_point.y, marker = "P", color = "r", markersize = 15, label="CV generated")
        ax[0].plot(self.var_point.x, self.var_point.y, marker = "P", color = "lime", markersize = 15, label = "Sensor generated", alpha=1)        
        ax[0].plot(init_to_var_x, init_to_var_y, linestyle = "dashed", color = "k")
        dist = str(round(sqrt((init_to_var_y[1]-init_to_var_y[0])**2 + (init_to_var_x[1]-init_to_var_x[0])**2),2))

        ax[0].text(x=np.mean(init_to_var_x), y=np.mean(init_to_var_y), s="".join([dist,unit]), ha="right", va="bottom", fontsize = 20)
        ax[0].legend(loc="upper left")


        ax[1].add_patch(can_side)
        line_x = [self.cm.x - self.radius, self.cm.x + self.radius]
        line_y = [self.cm.z, self.cm.z]
        line_y_2 = [self.var_point.z, self.var_point.z]
        line_x_3 = [self.cm.x, self.cm.x]
        line_y_3 = [self.cm.z, self.var_point.z]
        ax[1].plot(line_x, line_y, color = "r", linestyle = "dashed", linewidth = 3, label = "CV generated")
        ax[1].plot(line_x, line_y_2, color = "lime", linestyle = "dashed", linewidth = 3, label = "Sensor generated")
        ax[1].plot(line_x_3, line_y_3, color = "k", linestyle = "dashed", linewidth = 3)
        dist2 = str(round(abs(line_y_3[1]-line_y_3[0]),2))
        ax[1].text(x=np.mean(line_x_3), y=np.mean(line_y_3), s="".join([dist2,unit]), ha="right", va="bottom", fontsize = 20)
        ax[1].legend(loc="upper left")

        plt.show()



    
    

