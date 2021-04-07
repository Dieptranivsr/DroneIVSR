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
#include <mavros_msgs/SetMavFrame.h>
#include <angles/angles.h>
#include <eigen_conversions/eigen_msg.h>
#include <visualization_msgs/Marker.h>

#include <control_velpid/velpid_lib.h>
#include <control_velpid/pid_controller.h>

Eigen::Vector3d velocity;
float batt_percent, err_th = 0.2;

geometry_msgs::TwistStamped data_vel;
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	data_vel = *msg;
}

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
	ros::Subscriber vel_sub = td.subscribe<geometry_msgs::TwistStamped>
			("/mavros/local_position/velocity_local", 10, vel_cb);
	ros::Subscriber batt_sub = td.subscribe<sensor_msgs::BatteryState>
			("mavros/battery", 10, battery_cb);

	ros::Publisher local_pos_sp_pub = td.advertise<geometry_msgs::PoseStamped>
			("/mavros/setpoint_position/local", 10);
	ros::Publisher vel_sp_pub = td.advertise<geometry_msgs::TwistStamped>
			("/mavros/setpoint_velocity/cmd_vel", 10);

	ros::Publisher marker_pub = td.advertise<visualization_msgs::Marker>
	    	("visualization_marker", 10);

	ros::ServiceClient arming_client = td.serviceClient<mavros_msgs::CommandBool>
			("mavros/cmd/arming");
	ros::ServiceClient set_mode_client = td.serviceClient<mavros_msgs::SetMode>
			("mavros/set_mode");

	// the setpoint publishing rate MUST be faster than 2Hz
	int rate = 20;
	ros::Rate loop_rate(rate);

	double linvel_p_gain = 0.4;         //1.4, 0.8
	double linvel_i_gain = 0.00;        //-------- 0,1           0.2  #0.05
	double linvel_d_gain = 0.12;        //------------ 0.24, 0.2 0.4
	double linvel_i_max = 0.12;
	double linvel_i_min = -0.1;
	setup_livel_pid(linvel_p_gain, linvel_i_gain, linvel_d_gain, linvel_i_max, linvel_i_min);

    visualization_msgs::Marker points, line_strip, landmark;
    points.header.frame_id = line_strip.header.frame_id = landmark.header.frame_id = "map";
    points.header.stamp = line_strip.header.stamp = landmark.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = landmark.ns = "points_and_lines";
    points.action = line_strip.action = landmark.action = visualization_msgs::Marker::ADD;
    points.id = 0;
    line_strip.id = 1;
    landmark.id = 2;
    points.type = visualization_msgs::Marker::SPHERE_LIST;         // POINTS, SPHERE_LIST
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    landmark.type = visualization_msgs::Marker::POINTS;
    points.pose.orientation.w = line_strip.pose.orientation.w = landmark.pose.orientation.w = 1.0;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.05;         // 0.05, 0.1
    points.scale.y = 0.05;
    points.scale.z = 0.05;

    line_strip.scale.x = 0.03;
    line_strip.scale.x = 0.03;
    line_strip.scale.x = 0.03;

    landmark.scale.x = 0.1;
    landmark.scale.y = 0.1;
    landmark.scale.z = 0.1;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // Landmark is red
    landmark.color.r = 1.0;
    landmark.color.a = 1.0;

	// wait for FCU connection
	ROS_INFO("Waiting for FCU connection .");
	while(ros::ok() && current_state.connected){
		std::cout << ". ";
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

    ros::Time last_time = ros::Time::now();
    Eigen::Vector3d dest;
    Eigen::Vector3d current;
    int target = 1, mode = 1;
    bool stop;

    while( ros::ok())
    {
    	batt_percent = current_batt.percentage * 100;
    	ROS_INFO_STREAM("Current Battery: " << batt_percent << "%");

    	while( ros::ok() && mode == 1)
    	{
    		points.points.push_back(current_pose.pose.position);
    		line_strip.points.push_back(current_pose.pose.position);
    		marker_pub.publish(points);
    		marker_pub.publish(line_strip);

    		local_pos_sp_pub.publish(pose_A);
    		loop_rate.sleep();
    		ros::spinOnce();
    		if (_position_distance(current_pose, pose_A) < err_th)
    		{
    			mode = 2;
    		}
    	}

    	switch(target)
    	{
    	case 1:
    		pose_B.pose.position.x = value_A.x() + 3;
    		pose_B.pose.position.y = value_A.y() + 3;
    		pose_B.pose.position.z = value_A.z();
    		break;
    	case 2:
    		pose_B.pose.position.x = value_A.x() - 3;
    		pose_B.pose.position.y = value_A.y() + 3;
    		pose_B.pose.position.z = value_A.z();
    		break;
    	case 3:
    		pose_B.pose.position.x = value_A.x() - 3;
    		pose_B.pose.position.y = value_A.y() - 3;
    		pose_B.pose.position.z = value_A.z();
    		break;
    	case 4:
    		pose_B.pose.position.x = value_A.x() + 3;
    		pose_B.pose.position.y = value_A.y() - 3;
    		pose_B.pose.position.z = value_A.z();
    		break;
    	case 5:
    		pose_B.pose.position.x = value_A.x() + 3;
    		pose_B.pose.position.y = value_A.y() + 3;
    		pose_B.pose.position.z = value_A.z();
    		break;
    	}

    	landmark.points.push_back(pose_B.pose.position);
    	marker_pub.publish(landmark);
    	last_time = ros::Time::now();
    	stop = false;

    	while( ros::ok() && !stop)
    	{
    		batt_percent = current_batt.percentage * 100;
    		ROS_INFO_STREAM("Current Battery: " << batt_percent << "%");

    		tf::pointMsgToEigen(pose_B.pose.position, dest);
    		tf::pointMsgToEigen(current_pose.pose.position, current);
    		points.points.push_back(current_pose.pose.position);
    		line_strip.points.push_back(current_pose.pose.position);
    		marker_pub.publish(points);
    		marker_pub.publish(line_strip);

    		tf::vectorEigenToMsg(compute_linvel_effort(dest, current, last_time), vel_msg.twist.linear);
    		vel_sp_pub.publish(vel_msg);
    		last_time = ros::Time::now();

    		ros::spinOnce();
    		loop_rate.sleep();

    		if (_position_distance(current_pose, pose_B) < err_th )
    			stop = true;
    	}
    	++target;

    	if (target == 6)
    		break;
    }

    offb_set_mode.request.custom_mode = "AUTO.LAND";
    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
    	ROS_INFO("AUTO.LAND enabled");

	return 0;
}
