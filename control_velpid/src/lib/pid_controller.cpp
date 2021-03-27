#include <iostream>
#include <boost/algorithm/clamp.hpp>
#include <boost/algorithm/minmax.hpp>

#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <eigen3/Eigen/Dense>
#include <control_velpid/pid_controller.h>

void setup_livel_pid(double p_gain, double i_gain, double d_gain, double i_max, double i_min)
{
	// PID values
	p_gain_ = p_gain;
	i_gain_ = i_gain;
	d_gain_ = d_gain;

	// Min/max bounds for the integral windup
	i_min_ = i_min;
	i_max_ = i_max;
	
	//ROS_INFO("PID: %f, %f, %f", p_gain_, i_gain_, d_gain_);
}

double computeCommand_x(double error, ros::Duration dt)
{
	if (dt == ros::Duration(0.0) || std::isnan(error) || std::isinf(error))
		return 0.0;
	
	double error_dot = d_error_x;

	// Calculate the derivative error
	if (dt.toSec() > 0.0)
	{
		if (valid_p_error_last_x) {
			error_dot = (error - p_error_last_x) / dt.toSec();
	    }
	    p_error_last_x = error;
	    valid_p_error_last_x = true;
	}
	return computeCommandX(error, error_dot, dt);
}

double computeCommandX(double error, double error_dot, ros::Duration dt)
{
	double p_term, d_term, i_term;
	p_error_x = error; // this is error = target - state
	d_error_x = error_dot;

	if (dt == ros::Duration(0.0) || std::isnan(error) || std::isinf(error) || std::isnan(error_dot) || std::isinf(error_dot))
		return 0.0;

	// Calculate proportional contribution to command
	p_term = p_gain_ * p_error_x;

	// Calculate the integral of the position error
	i_error_x += dt.toSec() * p_error_x;
	
	if(i_gain_!=0)
	{
		// Prevent i_error_ from climbing higher than permitted by i_max_/i_min_
		boost::tuple<double, double> bounds = boost::minmax<double>(i_min_ / i_gain_, i_max_ / i_gain_);
		i_error_x = boost::algorithm::clamp(i_error_x, bounds.get<0>(), bounds.get<1>());
	}
	
	// Calculate integral contribution to command
	i_term = i_gain_ * i_error_x;
	
	// Limit i_term so that the limit is meaningful in the output
	i_term = boost::algorithm::clamp(i_term, i_min_, i_max_);
	
	// Calculate derivative contribution to command
	d_term = d_gain_ * d_error_x;
	
	// Compute the command
	cmd_x = p_term + i_term + d_term;
	
	return cmd_x;
}


double computeCommand_y(double error, ros::Duration dt)
{
	if (dt == ros::Duration(0.0) || std::isnan(error) || std::isinf(error))
		return 0.0;
	
	double error_dot = d_error_y;

	// Calculate the derivative error
	if (dt.toSec() > 0.0)
	{
		if (valid_p_error_last_y) {
			error_dot = (error - p_error_last_y) / dt.toSec();
	    }
	    p_error_last_y = error;
	    valid_p_error_last_y = true;
	}
	return computeCommandY(error, error_dot, dt);
}

double computeCommandY(double error, double error_dot, ros::Duration dt)
{
	double p_term, d_term, i_term;
	p_error_y = error; // this is error = target - state
	d_error_y = error_dot;

	if (dt == ros::Duration(0.0) || std::isnan(error) || std::isinf(error) || std::isnan(error_dot) || std::isinf(error_dot))
		return 0.0;

	// Calculate proportional contribution to command
	p_term = p_gain_ * p_error_y;

	// Calculate the integral of the position error
	i_error_y += dt.toSec() * p_error_y;
	
	if(i_gain_!=0)
	{
		// Prevent i_error_ from climbing higher than permitted by i_max_/i_min_
		boost::tuple<double, double> bounds = boost::minmax<double>(i_min_ / i_gain_, i_max_ / i_gain_);
		i_error_y = boost::algorithm::clamp(i_error_y, bounds.get<0>(), bounds.get<1>());
	}
	
	// Calculate integral contribution to command
	i_term = i_gain_ * i_error_y;
	
	// Limit i_term so that the limit is meaningful in the output
	i_term = boost::algorithm::clamp(i_term, i_min_, i_max_);
	
	// Calculate derivative contribution to command
	d_term = d_gain_ * d_error_y;
	
	// Compute the command
	cmd_y = p_term + i_term + d_term;
	
	return cmd_y;
}


double computeCommand_z(double error, ros::Duration dt)
{
	if (dt == ros::Duration(0.0) || std::isnan(error) || std::isinf(error))
		return 0.0;
	
	double error_dot = d_error_z;

	// Calculate the derivative error
	if (dt.toSec() > 0.0)
	{
		if (valid_p_error_last_z) {
			error_dot = (error - p_error_last_z) / dt.toSec();
	    }
	    p_error_last_z = error;
	    valid_p_error_last_z = true;
	}
	return computeCommandZ(error, error_dot, dt);
}

double computeCommandZ(double error, double error_dot, ros::Duration dt)
{
	double p_term, d_term, i_term;
	p_error_z = error; // this is error = target - state
	d_error_z = error_dot;

	if (dt == ros::Duration(0.0) || std::isnan(error) || std::isinf(error) || std::isnan(error_dot) || std::isinf(error_dot))
		return 0.0;

	// Calculate proportional contribution to command
	p_term = p_gain_ * p_error_z;

	// Calculate the integral of the position error
	i_error_z += dt.toSec() * p_error_z;
	
	if(i_gain_!=0)
	{
		// Prevent i_error_ from climbing higher than permitted by i_max_/i_min_
		boost::tuple<double, double> bounds = boost::minmax<double>(i_min_ / i_gain_, i_max_ / i_gain_);
		i_error_z = boost::algorithm::clamp(i_error_z, bounds.get<0>(), bounds.get<1>());
	}
	
	// Calculate integral contribution to command
	i_term = i_gain_ * i_error_z;
	
	// Limit i_term so that the limit is meaningful in the output
	i_term = boost::algorithm::clamp(i_term, i_min_, i_max_);
	
	// Calculate derivative contribution to command
	d_term = d_gain_ * d_error_z;
	
	// Compute the command
	cmd_z = p_term + i_term + d_term;
	
	return cmd_z;
}

// Function to bound the values of "v" between +/- "b"
float bound(float v, float b){
    if(v < -b)
        return -b;
    if(v > b)
        return b;
    return v;
}

Eigen::Vector3d compute_linvel_effort(Eigen::Vector3d goal, Eigen::Vector3d current, ros::Time last_time){
	double lin_vel_x = computeCommand_x(goal.x() - current.x(), ros::Time::now() - last_time);
	double lin_vel_y = computeCommand_y(goal.y() - current.y(), ros::Time::now() - last_time);
	double lin_vel_z = computeCommand_z(goal.z() - current.z(), ros::Time::now() - last_time);

	lin_vel_x = bound(lin_vel_x, MAX_vel);
	lin_vel_y = bound(lin_vel_y, MAX_vel);
	lin_vel_z = bound(lin_vel_z, MAX_vel);

	return Eigen::Vector3d(lin_vel_x, lin_vel_y, lin_vel_z);
}

/*
Eigen::Vector3d compute_linvel_effort(geometry_msgs::PoseStamped goal, geometry_msgs::PoseStamped current, ros::Time last_time){
	Eigen::Vector3d _goal, _current;
	tf::pointMsgToEigen(goal.pose.position, _goal);
	tf::pointMsgToEigen(current.pose.position, _current);

	double lin_vel_x = computeCommand_x(_goal(0) - _current(0), ros::Time::now() - last_time);
	double lin_vel_y = computeCommand_y(_goal(1) - _current(1), ros::Time::now() - last_time);
	double lin_vel_z = computeCommand_z(_goal(2) - _current(2), ros::Time::now() - last_time);

	lin_vel_x = bound(lin_vel_x, MAX_vel);
	lin_vel_y = bound(lin_vel_y, MAX_vel);
	lin_vel_z = bound(lin_vel_z, MAX_vel);

	return Eigen::Vector3d(lin_vel_x, lin_vel_y, lin_vel_z);
}
*/
