#!/usr/bin/env python3 

import rospy
from std_msgs.msg import Float32MultiArray
import matplotlib
matplotlib.use("TkAgg")
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from threading import Lock
from queue import Queue
import matplotlib.pyplot as plt
import math
from math import *

x_data = []
y_data = []
z_data  =[]
pwr_data = []
data_queue = Queue()
lock = Lock()

plt.ion()
fig = plt.figure()
ax=fig.add_subplot(111,projection='3d')
sc = ax.scatter([],[],[],c=[],cmap='jet')
cbar = plt.colorbar(sc,ax=ax)

ax.set_xlabel('East [m]')
ax.set_ylabel('North [m]')
ax.set_zlabel('Up [m]')
ax.set_title("Real-time 3D Data Plot")

def callback(data):

    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    print(f"{data.data}")

    north = (data.data[0])/100
    east = (data.data[1])/100
    up = -(data.data[2])/100
    pwr = 10*math.log10(data.data[3])

    with lock:
        data_queue.put((east,north,up,pwr))

def update_plot():
    global x_data,y_data,z_data,pwr_data

    while not rospy.is_shutdown():
        if not data_queue.empty():
            with lock:
                x,y,z,pwr = data_queue.get()
                x_data.append(x)
                y_data.append(y)
                z_data.append(z)
                pwr_data.append(pwr)

            ax.cla()

            sc = ax.scatter(x_data,y_data,z_data,c=pwr_data,cmap = 'jet')
            cbar.set_clim(min(pwr_data),max(pwr_data))

            ax.set_xlabel('East [m]')
            ax.set_ylabel('North [m]')
            ax.set_zlabel('Up [m]')
            ax.set_title("Real-time 3D Data Plot")

            plt.draw()
            plt.pause(1e-100)

if __name__ == '__main__':

    rospy.init_node("plotting_node")
    rospy.Subscriber("/sync_data",Float32MultiArray,callback)
    plt.show()
    while not rospy.is_shutdown():
        update_plot()
