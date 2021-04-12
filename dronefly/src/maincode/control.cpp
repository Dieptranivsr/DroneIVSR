#include <iostream>
#include <array>
#include <random>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <angles/angles.h>
#include <eigen_conversions/eigen_msg.h>

#include <dronefly/pid.h>
#include <dronefly/callback.h>
#include <dronefly/drone.h>

int main( int argc, char **argv)
{
	//
	// ROS INITIALIZATION
	//
	ros::init(argc, argv, "control_velocity");
	ros::NodeHandle td;

    ros::Subscriber state_sub = td.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

	ros::Subscriber local_pose_sub = td.subscribe<geometry_msgs::PoseStamped>
			("/mavros/local_position/pose", 10, local_pos_cb);
	ros::Subscriber batt_sub = td.subscribe<sensor_msgs::BatteryState>
			("mavros/battery", 10, battery_cb);

	ros::Subscriber getlocalpose_sub = td.subscribe<geometry_msgs::PoseStamped>
			("/drone/get/position_local", 10, getlocalpose_cb);

	ros::Publisher local_pos_sp_pub = td.advertise<geometry_msgs::PoseStamped>
			("/mavros/setpoint_position/local", 10);
	ros::Publisher vel_sp_pub = td.advertise<geometry_msgs::TwistStamped>
			("/mavros/setpoint_velocity/cmd_vel", 10);

	ros::ServiceClient arming_client = td.serviceClient<mavros_msgs::CommandBool>
			("mavros/cmd/arming");
	ros::ServiceClient set_mode_client = td.serviceClient<mavros_msgs::SetMode>
			("mavros/set_mode");

	// the setpoint publishing rate MUST be faster than 2Hz
	ros::Rate loop_rate(rate);

	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;

	double linvel_p_gain = 0.4;         //1.4, 0.8
	double linvel_i_gain = 0.00;        //-------- 0,1           0.2  #0.05
	double linvel_d_gain = 0.12;        //------------ 0.24, 0.2 0.4
	double linvel_i_max = 0.12;
	double linvel_i_min = -0.1;
	pidcontroller::PIDController pid;
	pid.setup_linvel_pid(linvel_p_gain, linvel_i_gain, linvel_d_gain, linvel_i_max, linvel_i_min);

    ros::Time last_time = ros::Time::now();
    geometry_msgs::TwistStamped vel_msg;
    geometry_msgs::PoseStamped goal;
    float err_th = 0.2;
    Eigen::Vector3d dest;
    Eigen::Vector3d current;
    int target = 1, mode = 0;
    bool stop;

    while( ros::ok())
    {
    	if (dest_pose!=goal && !stop)
    	{
    		goal = dest_pose;
    		stop = false;
    		tf::pointMsgToEigen(goal.pose.position, dest);
    		tf::pointMsgToEigen(current_pose.pose.position, current);

    		tf::vectorEigenToMsg(pid.compute_linvel_effort(dest, current, last_time), vel_msg.twist.linear);
    		vel_sp_pub.publish(vel_msg);
    		last_time = ros::Time::now();
    		ros::spinOnce();
    		loop_rate.sleep();

    		if (_position_distance(current_pose, goal) < err_th )
    			stop = true;
    	}
    }

    offb_set_mode.request.custom_mode = "AUTO.LAND";
    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
    	ROS_INFO("AUTO.LAND enabled");

	return 0;
}
