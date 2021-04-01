#include <iostream>
#include <array>
#include <random>
#include <vector>

#include <ros/ros.h>
#include <angles/angles.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMavFrame.h>
#include <sensor_msgs/BatteryState.h>
#include <eigen_conversions/eigen_msg.h>
#include <sensor_msgs/BatteryState.h>
#include <visualization_msgs/Marker.h>

#include <control_velpid/velpid_lib.h>
#include <control_velpid/pid_controller.h>

#include <control_velpid/captureGraph.h>

geometry_msgs::PoseStamped pose_A;
geometry_msgs::PoseStamped ps;
geometry_msgs::TwistStamped vs;
Eigen::Vector3d current, dest;

double err_th = 0.1;
double distance;
bool stop;

ros::Time last_time;

int main(int argc, char **argv)
{
	/*
	 * ROS INITIALIZATION
	 */
	ros::init(argc, argv, "test_velocity");
	ros::NodeHandle td;

	// Subcriber
	ros::Subscriber state_sub = td.subscribe<mavros_msgs::State>
			("mavros/state", 10, state_cb);
	ros::Subscriber local_pose_sub = td.subscribe<geometry_msgs::PoseStamped>
			("/mavros/local_position/pose", 10, local_pos_cb);
	ros::Subscriber batt_sub = td.subscribe<sensor_msgs::BatteryState>
			("mavros/battery", 10, battery_cb);

	ros::Publisher local_pos_sp_pub = td.advertise<geometry_msgs::PoseStamped>
			("/mavros/setpoint_position/local", 10);
	ros::Publisher vel_sp_pub = td.advertise<geometry_msgs::TwistStamped>
			("/mavros/setpoint_velocity/cmd_vel", 10);

	ros::Publisher marker_pub = td.advertise<visualization_msgs::Marker>
	    	("visualization_marker", 10);

	ros::ServiceClient set_mode_client = td.serviceClient<mavros_msgs::SetMode>
			("mavros/set_mode");
	ros::ServiceClient arming_client = td.serviceClient<mavros_msgs::CommandBool>
			("mavros/cmd/arming");

	threshold = threshold_definition();

	double rate = 20.0;

	double linvel_p_gain = 0.4;
	double linvel_i_gain = 0.2;
	double linvel_d_gain = 0.4;
	double linvel_i_max = 1.1;
	double linvel_i_min = -1.1;

	// Setup of the PID controllers
	setup_livel_pid(linvel_p_gain, linvel_i_gain, linvel_d_gain, linvel_i_max, linvel_i_min);

	//the setpoint publishing rate MUST be faster than 2Hz
	ros::Rate loop_rate(rate);

    visualization_msgs::Marker points, line_strip;
    points.header.frame_id = line_strip.header.frame_id = "map";
    points.header.stamp = line_strip.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = "points_and_lines";
    points.action = line_strip.action = visualization_msgs::Marker::ADD;
    points.id = 0;
    line_strip.id = 1;
    points.type = visualization_msgs::Marker::SPHERE_LIST;         // POINTS, SPHERE_LIST
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.05;         // 0.05, 0.1
    points.scale.y = 0.05;
    points.scale.z = 0.05;

    line_strip.scale.x = 0.03;
    line_strip.scale.x = 0.03;
    line_strip.scale.x = 0.03;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.r = 1.0;
    line_strip.color.a = 1.0;

    // wait for FCU connection
    ROS_INFO("Waiting for FCU connection .");
    while(ros::ok() && current_state.connected){
        //ROS_INFO_ONCE("Waiting for FCU connection ...");
    	std::cout << ".";
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

    ROS_INFO_STREAM("Are you run by PID - SQUARE ? (y/n)");
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
    	    ROS_INFO_STREAM("Are you run by PID - SQUARE ? (y/n)");
    	    std::cin >> a;
    	}
    	count++;
    }

    Eigen::Vector3d value_A;
    pose_A.pose.position.x = current_pose.pose.position.x;
    pose_A.pose.position.y = current_pose.pose.position.y;
    pose_A.pose.position.z = current_pose.pose.position.z + 3;
    tf::pointMsgToEigen(pose_A.pose.position, value_A);
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
    	pose_A.header.stamp = ros::Time::now();
    	local_pos_sp_pub.publish(pose_A);
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Ready");
    ros::Duration(3).sleep();

    // publish target, keep drone hovering
    int mode = 1;

	uint8_t pos_target = 1;

	ROS_INFO("Testing...");
	double origin;
	while (ros::ok()) {
    	switch(mode)
    	{
    	case 1:
    		points.points.push_back(current_pose.pose.position);
	        line_strip.points.push_back(current_pose.pose.position);
	        marker_pub.publish(points);
	        marker_pub.publish(line_strip);

    		ROS_INFO_STREAM("\nCurrent position: \n" << current_pose.pose.position);
    		batt_percent = current_batt.percentage * 100;
    		ROS_INFO_STREAM("Current Battery: " << batt_percent << "%");

    		local_pos_sp_pub.publish(pose_A);
    		loop_rate.sleep();
    		ros::spinOnce();

    		if( position_distance(current_pose, pose_A) == true)
    		{
    		   	ROS_WARN("Use PID velocity to fly from A to B");
    		    mode = 2;
    		}
    		break;
    	case 2:
			// motion routine
    		ROS_INFO_STREAM("\nCurrent position: \n" << current_pose.pose.position);
    		points.points.push_back(current_pose.pose.position);
	        line_strip.points.push_back(current_pose.pose.position);
	        marker_pub.publish(points);
	        marker_pub.publish(line_strip);

			switch (pos_target) {
			case 1:
				tf::pointEigenToMsg(pos_setpoint(value_A.x() + 3, value_A.y() + 3, 3), ps.pose.position);
				break;
			case 2:
				tf::pointEigenToMsg(pos_setpoint(value_A.x() - 3, value_A.y() + 3, 3), ps.pose.position);
				break;
			case 3:
				tf::pointEigenToMsg(pos_setpoint(value_A.x() - 3, value_A.y() - 3, 3), ps.pose.position);
				break;
			case 4:
				tf::pointEigenToMsg(pos_setpoint(value_A.x() + 3, value_A.y() - 3, 3), ps.pose.position);
				break;
			case 5:
				tf::pointEigenToMsg(pos_setpoint(value_A.x() + 3, value_A.y() + 3, 3), ps.pose.position);
				break;
			default:
				break;
			}

			last_time = ros::Time::now();
			stop = false;

			ROS_WARN("Next setpoint: accepted error threshold: %1.3f (m)", err_th);
			//
			// Defines the accepted threshold to the destination/target position before moving to the next setpoint.
			//
			while (!stop) {
	    		points.points.push_back(current_pose.pose.position);
		        line_strip.points.push_back(current_pose.pose.position);
		        marker_pub.publish(points);
		        marker_pub.publish(line_strip);

				loop_rate.sleep();
				ros::spinOnce();

				tf::pointMsgToEigen(ps.pose.position, dest);
				tf::pointMsgToEigen(current_pose.pose.position, current);

				ROS_INFO_STREAM("\nCurrent position: \n" << current_pose.pose.position);
				batt_percent = current_batt.percentage * 100;
				ROS_INFO_STREAM("Current Battery: " << batt_percent << "%");

				distance = sqrt((dest - current).x() * (dest - current).x() +
								(dest - current).y() * (dest - current).y() +
								(dest - current).z() * (dest - current).z());
				ROS_INFO_STREAM("Distance : " << distance << "(m)");
				if (distance <= err_th)
					stop = true;

				tf::vectorEigenToMsg(compute_linvel_effort(dest, current, last_time), vs.twist.linear);

				vel_sp_pub.publish(vs);

				last_time = ros::Time::now();
			}

			if (pos_target == 6) {
				ROS_INFO("Test complete!");
				ROS_INFO("Travel real time: %6.6f (s)", ros::Time::now().toSec() - origin);

				offb_set_mode.request.custom_mode = "AUTO.LAND";
				if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
					ROS_INFO("AUTO.LAND enabled");

				++pos_target;
			}
			else
				++pos_target;
			break;
    	}
		if (pos_target == 7)
			break;
	}

	return 0;
}
