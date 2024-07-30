#!/usr/bin/env python

import rospy
import geometry_msgs
import random
import time
import math
import matplotlib.pyplot as plt
import pickle

class TrajPoint:
    def __init__(self, px=0.0, py=0.0, pz=0.0, ox=0.0, oy=0.0, oz=0.0, ow=1.0) -> None:
        self.px = px
        self.py = py
        self.pz = pz
        self.ox = ox
        self.oy = oy
        self.oz = oz
        self.ow = ow

class Traj:
    def __init__(self, num_points=1) -> None:
        self.data = []
        self.num_points = num_points
        for i in range(self.num_points):
            self.data.append(TrajPoint())

class TrajectoryGenerator:
    def __init__(self, num_points = 1) -> None:
        self.num_points = num_points
        self.trajectory = Traj(num_points= self.num_points)
    
    def generate_circle(self, 
                        radius = 0.1, # meter
                        init_px=0, init_py=0, init_pz=0, 
                        init_ox=0, init_oy=0, init_oz=0, init_ow=1):
        for idx in range(self.num_points):
            ang = 2*math.pi*idx/self.num_points
            self.trajectory.data[idx].px = init_px + radius*math.cos(ang)
            self.trajectory.data[idx].py = init_py + radius*math.sin(ang)
            self.trajectory.data[idx].pz = init_pz
            self.trajectory.data[idx].ox = init_ox
            self.trajectory.data[idx].oy = init_oy
            self.trajectory.data[idx].oz = init_oz
            self.trajectory.data[idx].ow = init_ow
            print(self.trajectory.data[idx].px, self.trajectory.data[idx].py, self.trajectory.data[idx].pz)

    def visualize_2d_trajectory(self):
        x = []
        y = []
        for idx in range(self.num_points):
            x.append(self.trajectory.data[idx].px)
            y.append(self.trajectory.data[idx].py)

        plt.scatter(x, y, c ="blue") 
        plt.show()

    def save_traj(self, path):
        x = []
        y = []
        for idx in range(self.num_points):
            x.append(self.trajectory.data[idx].px)
            y.append(self.trajectory.data[idx].py)

        trajectory = {}
        trajectory["data"] = self.trajectory.data
        trajectory["num_points"] = self.trajectory.num_points
        with open(path+"/trajectory.pickle", 'wb') as file:
            pickle.dump(trajectory, file, protocol=pickle.HIGHEST_PROTOCOL)

tg = TrajectoryGenerator(50)
tg.generate_circle(radius=0.075,
                   init_px=0.17133348935796633, init_py = -0.2812875632299275, init_pz=-0.2747095401812456,
                   init_ox=0.7070666262443575, init_oy=-0.7071468204090466, init_oz=-0.00024981334063552845, init_ow=0.00031309757866667385)
# tg.visualize_2d_trajectory()
tg.save_traj("/home/hzx")