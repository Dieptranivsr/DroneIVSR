#include <iostream>

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/BatteryState.h>

#include <test_velocity/vel_lib.h>

/*
 * convert angle
 */
// convert angle from radian to degree
double degree(double _rad)
{
	return (_rad*180)/td_PI;
}

// convert angle from degree to radian
double radian(double _deg)
{
	return (_deg*td_PI)/180;
}

/*
 * CALL FUNCTIONS
 */
// state callback
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
	current_state = *msg;
}

// local pose callback
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	current_pose = *msg;
}

// battery status callback
void battery_cb(const sensor_msgs::BatteryState::ConstPtr& msg)
{
    current_batt = *msg;
}
//check position
bool position_distance(geometry_msgs::PoseStamped e1, geometry_msgs::PoseStamped e2)
{
	Eigen::Vector3d ps1, ps2;
	tf::pointMsgToEigen(e1.pose.position, ps2);
	tf::pointMsgToEigen(e2.pose.position, ps1);
	
	// get threshold [0-1] = 6.95113e-310
	/*
     * std::array<double, 10> threshold;
     * double err_th = threshold[rand() % threshold.size()];
     */
    
	double threshold = 0.1;       //0.01 m || 0.03(m) || 0.05m || 0.1 m
	ROS_DEBUG("Next setpoint: accepted error threshold: %1.3f", threshold);
	double distance = sqrt( (ps2 - ps1).x() * (ps2 - ps1).x() +
							(ps2 - ps1).y() * (ps2 - ps1).y() + 
							(ps2 - ps1).z() * (ps2 - ps1).z());
	
	if (distance <= threshold){
		//ROS_INFO("The goal was successful.");
		return true;
	}
	else{
		//ROS_WARN("Can not go to the goal.");
		//ROS_WARN("Differnce distance between current position and goal posisiton is %f", distance);
		return false;
	}
}

double distance(geometry_msgs::PoseStamped e1, geometry_msgs::PoseStamped e2)
{
	Eigen::Vector3d ps1, ps2;
	tf::pointMsgToEigen(e1.pose.position, ps2);
	tf::pointMsgToEigen(e2.pose.position, ps1);

	return(sqrt((ps2 - ps1).x() * (ps2 - ps1).x() +
				(ps2 - ps1).y() * (ps2 - ps1).y() + 
				(ps2 - ps1).z() * (ps2 - ps1).z()));
}

geometry_msgs::Twist compute_velocity(geometry_msgs::PoseStamped e1, geometry_msgs::PoseStamped e2, double timer1)
{
	Eigen::Vector3d ps1, ps2;
	tf::pointMsgToEigen(e1.pose.position, ps1);
	tf::pointMsgToEigen(e2.pose.position, ps2);
	
	std::cout << "Estimated flight time: ";
	timer1 = 8;
	std::cout << timer1 << std::endl;
	//std::cin >> timer1;
	
	geometry_msgs::Twist vel_calc;
	vel_calc.linear.x = 0;
	vel_calc.linear.y = 0;
	vel_calc.linear.z = 0;
	vel_calc.angular.x = 0;
	vel_calc.angular.y = 0;
	vel_calc.angular.z = 0;
	tf::vectorEigenToMsg(Eigen::Vector3d((ps2 - ps1).x()/timer1, (ps2 - ps1).y()/timer1, (ps2 - ps1).z()/timer1), vel_calc.linear);
	ROS_INFO("The speed of the velocity is (% f, %f, %f) m/s", vel_calc.linear.x, vel_calc.linear.y, vel_calc.linear.z);
	return vel_calc;
}
