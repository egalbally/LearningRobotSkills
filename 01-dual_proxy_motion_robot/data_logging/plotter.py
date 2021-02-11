#!/usr/bin/env python3

# read text files and plot them

import matplotlib.pyplot as plt
import numpy as np
import sys

# data file to read given as argument
if len(sys.argv) < 2:
	print("Give the name of the file to read as an argument\n")
	exit()

file = np.loadtxt(sys.argv[1] ,skiprows=1, delimiter=",")

time = file[:,0]/1000000
robot_ee_pos = file[:,1:4]
robot_ee_vel = file[:,4:7]
robot_proxy = file[:,7:10]
joint_angles = file[:,10:17]
joint_velocities = file[:,17:24]
joint_command_torques = file[:,24:31]
sensed_force_moments = file[:,31:37]
desired_force = file[:,37:]

plt.figure(0)
# plt.plot(time, robot_ee_pos)
# plt.plot(time, robot_proxy, '--')
plt.plot(sensed_force_moments)
plt.title("Robot position vs Proxy position", FontSize=20)
plt.xlabel("Time (s)", FontSize=16)
plt.legend(['x', 'y', 'z'])

plt.figure(1)
# plt.plot(time, robot_ee_vel)
plt.plot(joint_command_torques[:,5:])
plt.title("Robot velocity", FontSize=20)
plt.xlabel("Time (s)", FontSize=16)
plt.legend(['x', 'y', 'z'])

plt.show()