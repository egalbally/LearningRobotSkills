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
haptic_pos = file[:,1:4]
haptic_vel = file[:,4:7]
haptic_proxy = file[:,7:10]
haptic_command_forces = file[:,10:13]

plt.figure(0)
plt.plot(time, haptic_pos)
plt.plot(time, haptic_proxy, '--')
plt.title("Haptic position vs Proxy position", FontSize=20)
plt.xlabel("Time (s)", FontSize=16)
plt.legend(['x', 'y', 'z'])

plt.figure(1)
plt.plot(time, haptic_vel)
plt.title("Haptic velocity", FontSize=20)
plt.xlabel("Time (s)", FontSize=16)
plt.legend(['x', 'y', 'z'])

plt.show()
