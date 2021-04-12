#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

# callback method for state sub
current_state = State() 
offb_set_mode = SetMode
def state_cb(state):
    global current_state
    current_state = state

sending_dest = rospy.Publisher("/uav/sending/position_local", PoseStamped, queue_size=10)
state_sub = rospy.Subscriber("/mavros/state", State, state_cb)
arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

pose = PoseStamped()

def send_localposition():
    rospy.init_node('offb_node', anonymous=True)
    prev_state = current_state
    rate = rospy.Rate(20.0) # MUST be more then 2Hz

    last_request = rospy.get_rostime()
    while not rospy.is_shutdown():
        now = rospy.get_rostime()
        if pose and (now - last_request > rospy.Duration(10.)) :
            input_pose = raw_input("Enter new position : ").split()
            if input_pose :
                pose.pose.position.x = int(input_pose[0])
                pose.pose.position.y = int(input_pose[1])
                pose.pose.position.z = int(input_pose[2])
                last_request = rospy.get_rostime()
            rospy.loginfo("Sending pose")
            print('[ ' + str(pose.pose.position.x) + ' ' + str(pose.pose.position.y) + ' ' + str(pose.pose.position.z) + ']')
        pose.header.stamp = rospy.Time.now()
        sending_dest.publish(pose)
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    send_localposition()
'''
if __name__ == '__main__':
    try:
        send_localposition()
    except rospy.ROSInterruptException:
        pass
'''