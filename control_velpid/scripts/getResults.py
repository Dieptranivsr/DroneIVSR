#!/usr/bin/env python
from time import time
import rospy
import mavros
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped, TwistStamped
import mavros_msgs.msg


class dieptran():
    def __init__(self):
        rospy.init_node('listener_results', anonymous=True)
        self.rate = rospy.Rate(20.0)  # MUST be more then 2Hz
        mavros.set_namespace('mavros')


        state_sub = rospy.Subscriber(mavros.get_topic('state'), mavros_msgs.msg.State, self.state_cb)
        # "/mavros/setpoint_velocity/cmd_vel"
        vel_pub_sub = rospy.Subscriber(mavros.get_topic('setpoint_velocity', 'cmd_vel'), TwistStamped, self.vel_pub_cb)
        vel_local_sub = rospy.Subscriber(mavros.get_topic('local_position','velocity_local'), TwistStamped, self.vel_local_cb)

        self.current_state = mavros_msgs.msg.State()
        self.vel_pub = [0.0] * 4
        self.vel_local = [0.0] * 4

        self.vel_pub_x = []
        self.vel_pub_y = []
        self.vel_pub_z = []
        self.vel_local_x = []
        self.vel_local_y = []
        self.vel_local_z = []

        #rospy.spin()
        self.show()
        self.plot()

    def state_cb(self, topic):
        self.current_state.armed = topic.armed
        self.current_state.connected = topic.connected
        self.current_state.mode = topic.mode

    def show(self):
        rospy.loginfo(self.vel_local[0])

    def vel_pub_cb(self, topic):
        self.vel_pub[0] = topic.twist.linear.x
        self.vel_pub[1] = topic.twist.linear.y
        self.vel_pub[2] = topic.twist.linear.z
        rospy.loginfo("*****************************")
        rospy.loginfo("velocity PUB : " + str(self.vel_pub[0]))

    def vel_local_cb(self, topic):
        self.vel_local[0] = topic.twist.linear.x
        self.vel_local[1] = topic.twist.linear.y
        self.vel_local[2] = topic.twist.linear.z
        rospy.loginfo("*****************************")
        rospy.loginfo("velocity LOCAL : " + str(self.vel_local[0]))

    '''
    def plot(self):
        #rospy.loginfo("velocity : " + str(self.vel_local[0]))
        while not rospy.is_shutdown():
            if self.current_state.mode == "OFFBOARD" and self.current_state.armed and self.vel_pub != 0 and self.vel_local != 0 :
                self.vel_pub_x = np.append(self.vel_pub_x, self.vel_pub[0])
                self.vel_pub_y = np.append(self.vel_pub_y, self.vel_pub[1])
                self.vel_pub_z = np.append(self.vel_pub_z, self.vel_pub[2])
                self.vel_local_x = np.append(self.vel_local_x, self.vel_local[0])
                self.vel_local_y = np.append(self.vel_local_y, self.vel_local[1])
                self.vel_local_z = np.append(self.vel_local_z, self.vel_local[2])
            rospy.spin()
            #self.rate.sleep()

            if self.vel_pub != 0 and self.vel_local != 0 :
                break
    '''
    def plot(self):
        rospy.loginfo("velocity LOCAL : " + str(self.vel_local[0]))
        #while self.current_state.mode == "OFFBOARD" and self.current_state.armed and self.vel_pub != 0 and self.vel_local != 0 :
        while not rospy.is_shutdown():
            if self.vel_pub != 0 and self.vel_local != 0:
                self.vel_pub_x = np.append(self.vel_pub_x, self.vel_pub[0])
                self.vel_pub_y = np.append(self.vel_pub_y, self.vel_pub[1])
                self.vel_pub_z = np.append(self.vel_pub_z, self.vel_pub[2])
                self.vel_local_x = np.append(self.vel_local_x, self.vel_local[0])
                self.vel_local_y = np.append(self.vel_local_y, self.vel_local[1])
                self.vel_local_z = np.append(self.vel_local_z, self.vel_local[2])

            rospy.spin()
            self.rate.sleep()

            if not self.vel_pub and not self.vel_local :
                break

        #print(len(self.vel_pub_x))
        x = np.arange(0, len(self.vel_pub_x))
        plt.plot(x, self.vel_pub_x, 'b-', label='vel_pub_x')
        plt.plot(x, self.vel_pub_y, 'g-', label='vel_pub_y')
        plt.plot(x, self.vel_pub_z, 'r-', label='vel_pub_z')
        plt.plot(x, self.vel_local_x, 'c-', label='vel_local_x')
        plt.plot(x, self.vel_local_y, 'm-', label='vel_local_y')
        plt.plot(x, self.vel_local_z, 'y-', label='vel_local_z')
        plt.legend(loc='upper left')
        plt.xlabel('time')
        plt.ylabel('velocity')
        plt.savefig('books_read' + str(time()) + '.png')


if __name__ == "__main__":
    run = dieptran()
    run.plot()

    '''
    try:
        run.plot()
    except rospy.ROSInterruptException:
        pass
    '''


'''
# callback method for state sub
current_state = State()
def state_cb(state):
    global current_state
    current_state = state

global vel_pub_sub
def vel_pub_cb(vel_pub):
    vel_pub_sub = TwistStamped()
    vel_pub_sub.twist = vel_pub

global vel_local_sub
def vel_local_cb(vel_local):
    vel_local_sub = TwistStamped()
    vel_local_sub.twist = vel_local

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
    vel_local_sub = rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, vel_local_cb)

    vel_pub_x = []
    vel_pub_y = []
    vel_pub_z = []
    vel_local_x = []
    vel_local_y = []
    vel_local_z = []

    while not rospy.is_shutdown():
        #rospy.loginfo("vel_pub_x = %f", vel_pub_x)
        rospy.loginfo("Hiiiiiiiiiiiiiiiiiiiiiiiii.................")
        #if current_state.mode == "OFFBOARD" and current_state.armed and vel_pub_sub and vel_local_sub:
        if vel_pub_sub and vel_local_sub:
            vel_pub_x = np.append(vel_pub_x, vel_pub_sub.linear.x)
            vel_pub_y = np.append(vel_pub_y, vel_pub_sub.linear.y)
            vel_pub_z = np.append(vel_pub_z, vel_pub_sub.linear.z)
            vel_local_x = np.append(vel_local_x, vel_local_sub.linear.z)
            vel_local_y = np.append(vel_local_y, vel_local_sub.linear.z)
            vel_local_z = np.append(vel_local_z, vel_local_sub.linear.z)

            rospy.logerr("returned the invalid value %f", vel_pub_x)
            #rospy.loginfo("vel_pub_x = %f", vel_pub_x)
            #print("vel_pub_x = ", vel_pub_x)

        rospy.spin()
        rate.sleep()

        if not vel_pub_sub:
            break

    print(len(vel_pub_x))
    x = np.arange(0, len(vel_pub_x), 1)
    plt.plot(x, vel_pub_x, 'b-', label='vel_pub_x')
    plt.plot(x, vel_pub_y, 'g-', label='vel_pub_y')
    plt.plot(x, vel_pub_z, 'r-', label='vel_pub_z')
    plt.plot(x, vel_local_x, 'c-', label='vel_local_x')
    plt.plot(x, vel_local_y, 'm-', label='vel_local_y')
    plt.plot(x, vel_local_z, 'y-', label='vel_local_z')
    plt.legend(loc='upper left')
    plt.xlabel('time')
    plt.ylabel('velocity')
    plt.savefig('books_read'+str(time())+'.png')

if __name__ == '__main__':
    listener()

    try:
        listener()
    except rospy.ROSInterruptException:
        pass
    '''