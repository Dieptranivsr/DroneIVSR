#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode

# callback method for state sub
current_state = State()
def state_cb(state):
    global current_state
    current_state = state

def vel_pub_cb(vel_pub):
    global vel_pub_sub = TwistStamped()
    vel_pub_sub = vel_pub

def vel_local_cb(vel_local)
    global vel_local_sub = TwistStamped()
    vel_local_sub = vel_local

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener_results', anonymous=True)
    rate = rospy.Rate(20.0)  # MUST be more then 2Hz

    state_sub = rospy.Subscriber("/mavros/state", State, state_cb)
    vel_pub_sub = rospy.Subscriber("/mavros/setpoint_velocity/cmd_vel", TwistStamped, vel_pub_cb)
    vel_local_sub = rospy.Subscriber("/mavros/setpoint_velocity/cmd_vel", TwistStamped, vel_local_cb)

    vel_pub_x=[]
    vel_pub_y=[]
    vel_pub_z=[]
    vel_local_x = []

    while not rospy.is_shutdown():
        if current_state.mode == "OFFBOARD" and current_state.armed:
            vel_x = np.append(vel_x, vel_rs_sub.twist.linear.x)
            vel_y = np.append(vel_y, vel_rs_sub.twist.linear.y)
            vel_z = np.append(vel_z, vel_rs_sub.twist.linear.z)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass