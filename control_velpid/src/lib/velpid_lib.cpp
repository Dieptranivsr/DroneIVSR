#include <iostream>
#include <math.h>
#include <string>
#include <array>
#include <random>

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/BatteryState.h>
#include <eigen_conversions/eigen_msg.h>

#include <angles/angles.h>

#include <control_velpid/velpid_lib.h>

/*
 * CALL FUNCTIONS
 */
// local pose callback
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
	current_pose = *msg;
}

// state callback
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

// battery status callback
void battery_cb(const sensor_msgs::BatteryState::ConstPtr& msg)
{
    current_batt = *msg;
}

// Gaussian noise generator for accepted position threshold
std::array<double, 100> threshold_definition(){
	std::random_device rd;
	std::mt19937 gen(rd());
	std::array<double, 100> th_values;

	std::normal_distribution<double> th(0.1f,0.05f);

	for (auto &value : th_values) {
		value = th(gen);
	}
	return th_values;
}

//check position
bool position_distance(geometry_msgs::PoseStamped e1, geometry_msgs::PoseStamped e2)
{
	Eigen::Vector3d ps1, ps2;
	tf::pointMsgToEigen(e1.pose.position, ps2);
	tf::pointMsgToEigen(e2.pose.position, ps1);

	double threshold = 0.1;       //0.01 m || 0.03(m) || 0.05m || 0.1 m
	ROS_DEBUG("Next setpoint: accepted error threshold: %1.3f", threshold);
	double distance = sqrt( (ps2 - ps1).x() * (ps2 - ps1).x() +
							(ps2 - ps1).y() * (ps2 - ps1).y() + 
							(ps2 - ps1).z() * (ps2 - ps1).z());
	
	if (distance <= threshold){
		return true;
	}
	else{
		return false;
	}
}

// Defines single position setpoint
Eigen::Vector3d pos_setpoint(int tr_x, int tr_y, int tr_z){
	// Give possibility to user define amplitude of movement (square corners coordinates)
	ROS_WARN("Defines single position setpoint : ");
	std::cout << "(" << tr_x << ", " << tr_y << ", " << tr_z << ")" << std::endl;
	return Eigen::Vector3d(tr_x * 2.0f, tr_y * 2.0f, tr_z * 1.0f);	// meters
}

// Defines circle path
Eigen::Vector3d circle_shape(int angle){
	// Give possibility to user define amplitude of movement (circle radius)
	double r = 5.0f;	// 5 meters radius

	return Eigen::Vector3d(r * cos(angles::from_degrees(angle)), r * sin(angles::from_degrees(angle)), 1.0f);
}


// Defines Gerono lemniscate path
Eigen::Vector3d eight_shape(int angle){
	// Give possibility to user define amplitude of movement (vertical tangent size)
	double a = 5.0f;	// vertical tangent with 5 meters size

	return Eigen::Vector3d(a * cos(angles::from_degrees(angle)), a * sin(angles::from_degrees(angle)) * cos(angles::from_degrees(angle)), 2.0f);
}

// Defines ellipse path
Eigen::Vector3d ellipse_shape(int angle){
	// Give possibility to user define amplitude of movement (tangent sizes)
	double a = 5.0f;	// major axis
	double b = 2.0f;	// minor axis

	// rotation around y-axis
	return Eigen::Vector3d(a * cos(angles::from_degrees(angle)), 0.0f, 2.5f + b * sin(angles::from_degrees(angle)));
}






