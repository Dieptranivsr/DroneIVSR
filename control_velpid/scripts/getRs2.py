#!/usr/bin/env python

from time import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

import rospy
import math
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import BatteryState

# callback method for state sub
current_state = State()
def state_cb(state):
    global current_state
    current_state = state

# callback method for velocity pub sub
velocity_pub = TwistStamped()
def velocity_pub_cb(vel1):
    global velocity_pub
    velocity_pub = vel1

# callback method for velocity local sub
velocity_local = TwistStamped()
def velocity_local_cb(vel2):
    global velocity_local
    velocity_local = vel2


state_sub = rospy.Subscriber("/mavros/state", State, state_cb)
arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

velocity_local_sub = rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, velocity_local_cb)
velocity_pub_sub = rospy.Subscriber("/mavros/setpoint_velocity/cmd_vel", TwistStamped, velocity_pub_cb)

def getResultVel():
    rospy.init_node('offb_node', anonymous=True)
    rate = rospy.Rate(20.0)  # MUST be more then 2Hz

    rospy.loginfo("Waiting for Bag file connection .")

    vel_pub_x = []
    vel_pub_y = []
    vel_pub_z = []
    vel_local_x = []
    vel_local_y = []
    vel_local_z = []

    while not rospy.is_shutdown():
        if current_state.mode == "OFFBOARD" and current_state.armed and velocity_pub.twist.linear.x != 0:
            vel_pub_x = np.append(vel_pub_x, velocity_pub.twist.linear.x)
            vel_pub_y = np.append(vel_pub_y, velocity_pub.twist.linear.y)
            vel_pub_z = np.append(vel_pub_z, velocity_pub.twist.linear.z)
            vel_local_x = np.append(vel_local_x, velocity_local.twist.linear.x)
            vel_local_y = np.append(vel_local_y, velocity_local.twist.linear.y)
            vel_local_z = np.append(vel_local_z, velocity_local.twist.linear.z)

        rate.sleep()

    print(len(vel_pub_x))
    x = np.arange(0, len(vel_pub_x), 1)

    plt.plot(x, vel_pub_x, 'b-', label='vel_pub_x')
    plt.plot(x, vel_pub_y, 'g-', label='vel_pub_y')
    plt.plot(x, vel_pub_z, 'r-', label='vel_pub_z')
    plt.plot(x, vel_local_x, 'c-', label='vel_local_x')
    plt.plot(x, vel_local_y, 'm-', label='vel_local_y')
    plt.plot(x, vel_local_z, 'y-', label='vel_local_z')
    '''
    plt.legend(loc='upper left')
    plt.xlabel('time')
    plt.ylabel('velocity')
    plt.savefig('Velocity_result'+str(time())+'.jpeg', dpi=400,  figsize=(20,6))
    rospy.loginfo("Complete get data from bag file")
    '''
    plt.legend( prop={'size': 20})
    plt.xlabel('Time ', fontsize=24)
    plt.ylabel('Velocity (m/s)', fontsize=24)
    plt.grid(True)
    title = "PID Velocity Estimated  and Local Position Velocity"
    plt.suptitle(title, fontsize=36)
    plt.tick_params(axis='both', which='major', labelsize=18)
    plt.ylim(-3.5, 3.5)
    axes = plt.gca()
    ylims = axes.get_ylim()

    mng = plt.get_current_fig_manager()
    mng.resize(*mng.window.maxsize())
    plt.show()

    '''
    plt.plot(x, vel_pub_x, 'b-', label='vel_pub_x')
    plt.plot(x, vel_pub_y, 'g-', label='vel_pub_y')
    plt.plot(x, vel_pub_z, 'r-', label='vel_pub_z')
    plt.legend(loc='upper left')
    plt.xlabel('time')
    plt.ylabel('velocity publish')
    plt.savefig('Velocity_publish'+str(time())+'.jpeg', dpi=2000)

    
    plt.plot(x, vel_local_x, 'c-', label='vel_local_x')
    plt.plot(x, vel_local_y, 'm-', label='vel_local_y')
    plt.plot(x, vel_local_z, 'y-', label='vel_local_z')
    plt.legend(loc='upper left')
    plt.xlabel('time')
    plt.ylabel('velocity local')
    plt.savefig('Velocity_local'+str(time())+'.jpeg', dpi=2000)
    '''
    #rospy.loginfo("Complete get data from bag file")

    rospy.spin()

if __name__ == '__main__':
    try:
        getResultVel()
    except rospy.ROSInterruptException:
        pass