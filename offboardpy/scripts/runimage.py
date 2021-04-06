import rosbag
import rospy
import sys
import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import csv

#filename_vel = sys.argv[0]
filename_vel = '/home/dieptran/catkin_ws/2021-04-04-15-18-16.bag'

topicdata = []
patches = []
colors = ["#CC0000", "#0000CC", "#00994C"]
labels = ["Vel publish x", "Vel publish y", "Vel publish z"]
linewidths = [1.5, 1.5, 1.5]
#give your plot a title
title = "PID Velocity Estimated  and Local Position Velocity"

bag1 = rosbag.Bag(filename_vel)
bag2 = rosbag.Bag(filename_vel)
tempdata1=np.array([[0],[0]])
tempdata2=np.array([[0],[0]])
tempdata3=np.array([[0],[0]])

for topic, msg, t in bag1.read_messages(topics="/mavros/setpoint_velocity/cmd_vel"):
	#print(msg.pose.pose.position.x)
	#case switch how to access x coordinate for all topics
	if topic == "/mavros/setpoint_velocity/cmd_vel" and msg.twist.linear.x > 0:
		tempdata1 = np.append(tempdata1, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs],[msg.twist.linear.x]], axis=1)
		tempdata2 = np.append(tempdata2, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs],[msg.twist.linear.y]], axis=1)
		tempdata3 = np.append(tempdata3, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs],[msg.twist.linear.z]], axis=1)
        #print(msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs)
	#topicdata
topicdata.append(tempdata1)
topicdata.append(tempdata2)
topicdata.append(tempdata3)

for idx, topic in enumerate(topicdata):
	plt.plot(topic[0,1:], topic[1,1:], color=colors[idx], linewidth=linewidths[idx])
	patches.append(mpatches.Patch(color=colors[idx], label=labels[idx]))

plt.legend(handles=patches, prop={'size': 20})
plt.xlabel('Time (s)', fontsize=24)
plt.ylabel('Velocity (m/s)', fontsize=24)
plt.grid(True)
plt.suptitle(title, fontsize=36)
plt.tick_params(axis='both', which='major', labelsize=18)
plt.ylim(-3.5,3.5)
axes = plt.gca()
ylims = axes.get_ylim()

mng = plt.get_current_fig_manager()
mng.resize(*mng.window.maxsize())
plt.show()