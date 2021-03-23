#include <iostream>
#include <boost/algorithm/clamp.hpp>
#include <boost/algorithm/minmax.hpp>

#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <eigen3/Eigen/Dense>
#include <control_velpid/pid_controllerv2.h>


double quaternion_to_euler(double x, double y, double z, double w)
{
	double t0, t1, t2, t3, t4;
	double X,Y,Z;
	t0 = 2.0*(w*x + y*z);
	t1 = 1.0 - 2.0*(x*x + y*y);
	X = atan(t0/t1)*180/M_PI;

	t2 = 2*(w*y - z*x);
	if (t2 > 1)
		t2 = 1;
	else if (t2 <-1)
		t2 = -1;
	else
		t2 = t2;
	Y = asin(t2)*180/M_PI;

	t3 = 2.0*(w*z + x*y);
	t4 = 1.0 - 2*(y*y + z*z);
	Z = atan(t3/t4)*180/M_PI;

	return X, Y, Z;
}

/*
double compute_linvel_effort_v2(geometry_msgs::PoseStamped goal, geometry_msgs::PoseStamped current, ros::Time last_time){
	// Local machine time
	_dt = ros::Time::now() - last_time;

	tf::pointMsgToEigen(goal.pose.position, _goal);
	tf::pointMsgToEigen(current.pose.position, _current);

	// Estimate error in world (Initial) Frame
	distance_error = _goal - _current;
	distance_integral = distance_integral + distance_error * _dt.toSec();
	distance_derivative = (distance_error - previous_distance_error)/_dt.toSec();

	// Store
	previous_distance_error = distance_error;

	//vel_world = Kp.dot(distance_error) + Ki.dot(distance_integral) + Kd.dot(distance_derivative);
	vel_world(0) = Kp(0)*distance_error(0) + Ki(0)*distance_integral(0) + Kd(0)*distance_derivative(0);
	vel_world(1) = Kp(1)*distance_error(1) + Ki(1)*distance_integral(1) + Kd(1)*distance_derivative(1);
	vel_world(2) = Kp(2)*distance_error(2) + Ki(2)*distance_integral(2) + Kd(2)*distance_derivative(2);

	roll, pitch, yaw = quaternion_to_euler(current.pose.orientation.x, current.pose.orientation.y, current.pose.orientation.z, current.pose.orientation.w);
	yaw_rate_world = (pose_goal(3) - yaw)*yaw_gain;

	Rbv1 << cos(yaw), sin(yaw), 0;
	Rbv2 << -sin(yaw), cos(yaw), 0;
	Rbv3 << 0, 0, 1;

	vel_body(0) = Rbv1(0)*vel_world(0) + Rbv1(1)*vel_world(1) + Rbv1(2)*vel_world(2);
	vel_body(1) = Rbv2(0)*vel_world(0) + Rbv2(1)*vel_world(1) + Rbv2(2)*vel_world(2);
	vel_body(2) = Rbv3(0)*vel_world(0) + Rbv3(1)*vel_world(1) + Rbv3(2)*vel_world(2);

	return (vel_body(0), vel_body(1), vel_body(2), yaw_rate_world);
}
*/


double compute_linvel_effort_v2(geometry_msgs::PoseStamped goal, geometry_msgs::PoseStamped current, ros::Time last_time){
	// Local machine time
	_dt = ros::Time::now() - last_time;

	tf::pointMsgToEigen(goal.pose.position, _goal);
	tf::pointMsgToEigen(current.pose.position, _current);

	// Estimate error in world (Initial) Frame
	distance_error = _goal - _current;
	distance_integral = distance_integral + distance_error * _dt.toSec();
	distance_derivative = (distance_error - previous_distance_error)/_dt.toSec();

	// Store
	previous_distance_error = distance_error;

	//vel_world = Kp.dot(distance_error) + Ki.dot(distance_integral) + Kd.dot(distance_derivative);
	vel_world(0) = Kp(0)*distance_error(0) + Ki(0)*distance_integral(0) + Kd(0)*distance_derivative(0);
	vel_world(1) = Kp(1)*distance_error(1) + Ki(1)*distance_integral(1) + Kd(1)*distance_derivative(1);
	vel_world(2) = Kp(2)*distance_error(2) + Ki(2)*distance_integral(2) + Kd(2)*distance_derivative(2);

	//roll, pitch, yaw = quaternion_to_euler(current.pose.orientation.x, current.pose.orientation.y, current.pose.orientation.z, current.pose.orientation.w);
	//yaw_rate_world = (pose_goal(3) - yaw)*yaw_gain;

	//return (vel_world(0), vel_world(1), vel_world(2), yaw_rate_world);
	return (vel_world(0), vel_world(1), vel_world(2));
}

