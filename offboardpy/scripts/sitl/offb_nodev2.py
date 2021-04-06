#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

from visualization_msgs.msg import Marker

# callback method for state sub
current_state = State() 
offb_set_mode = SetMode
def state_cb(state):
    global current_state
    current_state = state


# callback method for pose_local_cb
current_pose = PoseStamped()
def pose_local_cb(pose):
    global current_pose
    current_pose = pose

points = Marker()
line_strip = Marker()
landmark = Marker()
points.header.frame_id = line_strip.header.frame_id = landmark.header.frame_id = "map"
#points.header.stamp = line_strip.header.stamp = landmark.header.stamp = rospy.Time.now()
points.ns = line_strip.ns = landmark.ns = "points_and_lines"
points.action = line_strip.action = landmark.action = Marker.ADD
points.id = 0
line_strip.id = 1
landmark.id = 2
points.type = Marker.SPHERE_LIST
line_strip.type = Marker.LINE_STRIP
landmark.type = Marker.POINTS
points.pose.orientation.w = line_strip.pose.orientation.w = landmark.pose.orientation.w = 1.0

# POINTS markers use x and y scale for width/height respectively
points.scale.x = 0.05         # 0.05, 0.1
points.scale.y = 0.05
points.scale.z = 0.05

line_strip.scale.x = 0.03
line_strip.scale.x = 0.03
line_strip.scale.x = 0.03

landmark.scale.x = 0.1
landmark.scale.y = 0.1
landmark.scale.z = 0.1

# Points are green
points.color.g = 1.0
points.color.a = 1.0

# Line strip is blue
line_strip.color.b = 1.0
line_strip.color.a = 1.0

# Landmark is red
landmark.color.r = 1.0
landmark.color.a = 1.0

state_sub = rospy.Subscriber("/mavros/state", State, state_cb)
pose_local_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_local_cb)

local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)

arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

pose_A = PoseStamped()
pose_A.pose.position.x = 0
pose_A.pose.position.y = 0
pose_A.pose.position.z = 5
landmark.points.push_back(pose_A.pose.position)
marker_pub.publish(landmark)

def sitl_control():
    rospy.init_node('offb_node', anonymous=True)
    prev_state = current_state
    rate = rospy.Rate(20.0) # MUST be more then 2Hz

    # send a few setpoints before starting
    for i in range(100):
        local_pos_pub.publish(pose_A)
        rate.sleep()
    
    # wait for FCU connection
    while not current_state.connected:
        rate.sleep()

    last_request = rospy.get_rostime()
    while not rospy.is_shutdown():
        now = rospy.get_rostime()
        if current_state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5.)):
            set_mode_client(base_mode=0, custom_mode="OFFBOARD")
            last_request = now 
        else:
            if not current_state.armed and (now - last_request > rospy.Duration(5.)):
               arming_client(True)
               last_request = now 

        # older versions of PX4 always return success==True, so better to check Status instead
        if prev_state.armed != current_state.armed:
            rospy.loginfo("Vehicle armed: %r" % current_state.armed)
        if prev_state.mode != current_state.mode: 
            rospy.loginfo("Current mode: %s" % current_state.mode)
        prev_state = current_state

        # Update timestamp and publish pose 
        pose_A.header.stamp = rospy.Time.now()
        local_pos_pub.publish(pose_A)

        rate.sleep()

        points.points.push_back(current_pose.pose.position)
        marker_pub.publish(points)
        line_strip.points.push_back(current_pose.pose.position)
        marker_pub.publish(line_strip)
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
        sitl_control()
    except rospy.ROSInterruptException:
        pass
