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

Eigen::Vector3d velocity;
float batt_percent, err_th = 0.2;

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
	int rate = 20;

	// wait for FCU connection
	ROS_INFO("Waiting for FCU connection .");
	while(ros::ok() && current_state.connected){
		//std::cout << ". ";
		ros::spinOnce();
		loop_rate.sleep();
	}
	ROS_INFO("FCU connected");

	// check current pose
	float batt_percent;
	for(int i = 100; ros::ok() && i > 0; --i){
		ROS_INFO_STREAM("\nCurrent position: \n" << current_pose.pose.position);

		float batt_percent = current_batt.percentage * 100;
		ROS_INFO_STREAM("Current Battery: " << batt_percent << "%");

		ros::spinOnce();
		loop_rate.sleep();
	}

	mavros_msgs::SetMode offb_set_mode;
	geometry_msgs::PoseStamped pose_A;
	geometry_msgs::PoseStamped pose_B;
	geometry_msgs::TwistStamped vel_msg;

	Eigen::Vector3d value_A;
	pose_A.pose.position.x = current_pose.pose.position.x;
	pose_A.pose.position.y = current_pose.pose.position.y;
	pose_A.pose.position.z = current_pose.pose.position.z + 5;
	landmark.points.push_back(pose_A.pose.position);
	marker_pub.publish(landmark);
	tf::pointMsgToEigen(pose_A.pose.position, value_A);

	ROS_INFO_STREAM("Do you fly (using PID) ? (y/n)");
	char a[100];
	int count = 0;
	std::cin >> a;
	while (1)
	{
		if (count > 10)
			ros::shutdown();

		if (strcmp(a, "y") == 0||strcmp(a, "Y") == 0)
			break;
		else
		{
			ROS_WARN("Can you retype your choice ?");
			ROS_INFO_STREAM("Do you fly (using PID) ? (y/n)");
			std::cin >> a;
		}
		count++;
	}

    while( ros::ok() && !stop)
    {
    	local_pos_sp_pub.publish(pose_A);
    	loop_rate.sleep();
    	ros::spinOnce();
    	if (_position_distance(current_pose, pose_A) < err_th)
    		stop = true;

        batt_percent = current_batt.percentage * 100;
        ROS_INFO_STREAM("Current Battery: " << batt_percent << "%");

    	points.points.push_back(current_pose.pose.position);
    	line_strip.points.push_back(current_pose.pose.position);
    	marker_pub.publish(points);
    	marker_pub.publish(line_strip);
    }

	ros::Time last_request = ros::Time::now();
    while( ros::ok())
    {
    	batt_percent = current_batt.percentage * 100;
    	ROS_INFO_STREAM("Current Battery: " << batt_percent << "%");

    	if(current_state.mode == "OFFBOARD" && current_state.armed && ros::Time::now() - last_request > ros::Duration(2.0))
    	{
    		ROS_INFO("Drone is flying");
    		last_request = ros::Time::now();
    	}
    }

	return 0;
}
