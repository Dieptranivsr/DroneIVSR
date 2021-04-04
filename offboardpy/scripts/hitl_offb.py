#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import BatteryState

# callback method for state sub
current_state = State()
offb_set_mode = SetMode

# callback method for battery_cb
current_batt = BatteryState()

# callback method for pose_local_cb
current_pose = PoseStamped()

def state_cb(state):
    global current_state
    current_state = state

def battery_cb(battery):
    global current_batt
    current_batt = battery

def pose_local_cb(pose):
    global current_pose
    current_pose = pose

local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
state_sub = rospy.Subscriber("/mavros/state", State, state_cb)
arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

pose_A = PoseStamped()
pose_A.pose.position.x = 0
pose_A.pose.position.y = 0
pose_A.pose.position.z = 5

batte_sub = rospy.Subscriber("/mavros/battery", BatteryState, battery_cb)
pose_local_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_local_cb)

def hitl_positioncontrol():
    rospy.init_node('offb_node', anonymous=True)
    prev_state = current_state
    rate = rospy.Rate(20.0)  # MUST be more then 2Hz

    rospy.loginfo("Waiting for FCU connection .")
    while not current_state.connected:
        print(" .")
        rate.sleep()

    rospy.loginfo("FCU connected")

    #for i in range(100) and not rospy.is_shutdown():
    for i in range(100):
        batt_percent = current_batt.percentage * 100;
        rospy.loginfo("Current Battery : " + str(batt_percent) + "%")

        rate.sleep()

    rospy.loginfo("Are you run HITL OFFBOARD ? (y/n)")

    x = raw_input()
    for i in range(100):
        if i > 10:
            rospy.signal_shutdown("SHUTDOWN")
        if x == "Y" or x == "y":
            break
        else:
            rospy.logwarn("Can you retype your choice ?")
            rospy.loginfo("Are you run HITL OFFBOARD ? (y/n)")
            x = raw_input()

    #for i in range(100) and not rospy.is_shutdown():
    for i in range(100):
        pose_A.header.stamp = rospy.Time.now()
        local_pos_pub.publish(pose_A)

        rate.sleep()

    rospy.loginfo("Ready")
    rospy.sleep(3.)

    while not rospy.is_shutdown():
        # Update timestamp and publish pose
        pose_A.header.stamp = rospy.Time.now()
        local_pos_pub.publish(pose_A)

        rate.sleep()

        rospy.loginfo("Current Position:")
        print("     x : " + str(current_pose.pose.position.x) + " (m)")
        print("     y : " + str(current_pose.pose.position.y) + " (m)")
        print("     z : " + str(current_pose.pose.position.z) + " (m)")
        p = math.sqrt((current_pose.pose.position.x - pose_A.pose.position.x ) * (current_pose.pose.position.x - pose_A.pose.position.x ) +
            (current_pose.pose.position.y - pose_A.pose.position.y ) * (current_pose.pose.position.y - pose_A.pose.position.y ) +
            (current_pose.pose.position.z - pose_A.pose.position.z ) * (current_pose.pose.position.z - pose_A.pose.position.z ) )
        print("[tuandiep] distance = " + str(p) + " (m)")
        if p < 0.05:
            set_mode_client(base_mode=0, custom_mode="AUTO.LAND")
            break

    rospy.spin()

if __name__ == '__main__':
    try:
        hitl_positioncontrol()
    except rospy.ROSInterruptException:
        pass
