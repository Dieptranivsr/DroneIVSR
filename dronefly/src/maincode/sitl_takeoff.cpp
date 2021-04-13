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

float batt_percent;

int main( int argc, char **argv)
{
	//
	// ROS INITIALIZATION
	//
	ros::init(argc, argv, "takeoff");
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

	ros::ServiceClient arming_client = td.serviceClient<mavros_msgs::CommandBool>
			("mavros/cmd/arming");
	ros::ServiceClient set_mode_client = td.serviceClient<mavros_msgs::SetMode>
			("mavros/set_mode");

	// the setpoint publishing rate MUST be faster than 2Hz
	ros::Rate loop_rate(rate);

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

		batt_percent = current_batt.percentage * 100;
		ROS_INFO_STREAM("Current Battery: " << batt_percent << "%");

		ros::spinOnce();
		loop_rate.sleep();
	}

	geometry_msgs::PoseStamped pose_A;
	ros::Time last_request = ros::Time::now();

	Eigen::Vector3d value_A;
	pose_A.pose.position.x = current_pose.pose.position.x;
	pose_A.pose.position.y = current_pose.pose.position.y;
	pose_A.pose.position.z = current_pose.pose.position.z + 5;

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

    while( ros::ok())
    {
    	if (!dest_pose.pose.position.x && !dest_pose.pose.position.y && !dest_pose.pose.position.z)
    	{
    		local_pos_sp_pub.publish(pose_A);
    		loop_rate.sleep();
    		ros::spinOnce();
    		continue;
    	}

    	// ros::Time last_request = ros::Time::now();
    	// if( ros::Time::now() - last_request > ros::Duration(10.0))
    	if (_position_distance(current_pose, pose_A, false) < err_th)
    	{
    		batt_percent = current_batt.percentage * 100;
    		ROS_INFO_STREAM("Current Battery: " << batt_percent << "%");
    		std::cout << "[POSITION] CURRENT POSE/n" <<  current_pose << std::endl;

    		if (ros::Time::now() - last_request > ros::Duration(10.0))
    		{
    			ROS_INFO("[NOTIFICATION] Drone is flying");
    			last_request = ros::Time::now();
    		}
    	}

    	loop_rate.sleep();
		ros::spinOnce();
    }

	return 0;
}
