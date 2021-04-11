#include <iostream>
#include <boost/algorithm/clamp.hpp>
#include <boost/algorithm/minmax.hpp>

#include <ros/ros.h>

#include <dronefly/compute_pid.h>

using namespace computepid;

PID::PID(double p, double i, double d, double imax, double imin)
{
	// PID values
	p_gain = p;
	i_gain = i;
	d_gain = d;

	// Min/max bounds for the integral windup
	i_max = imax;
	i_min = imin;

	//ROS_INFO("PID: %f, %f, %f", p_gain_, i_gain_, d_gain_);
}

PID::~PID()
{}

void PID::initPID(double p, double i, double d, double imax, double imin)
{
	// PID values
	p_gain = p;
	i_gain = i;
	d_gain = d;

	// Min/max bounds for the integral windup
	i_max = imax;
	i_min = imin;

	//ROS_INFO("PID: %f, %f, %f", p_gain_, i_gain_, d_gain_);
}

double PID::computeCommand(double error, ros::Duration dt)
{
	if (dt == ros::Duration(0.0) || std::isnan(error) || std::isinf(error))
		return 0.0;

	double error_dot = d_error;

	// Calculate the derivative error
	if (dt.toSec() > 0.0)
	{
		if (valid_p_error_last)
			error_dot = (error - p_error_last)/dt.toSec();
		p_error_last = error;
		valid_p_error_last = true;
	}

	return computeCommand(error, error_dot, dt);
}

double PID::computeCommand(double error, double error_dot, ros::Duration dt)
{
	double p_term, d_term, i_term;
	p_error = error;                 // this is error = target - state
	d_error = error_dot;

	if (dt == ros::Duration(0.0) || std::isnan(error) || std::isinf(error) || std::isnan(error_dot) || std::isinf(error_dot))
		return 0.0;

	// Calculate proportional contribution to command
	p_term = p_gain * p_error;

	// Calculate the integral of the position error
	i_error += dt.toSec() * p_error;

	if(i_gain!=0)
	{
		// Prevent i_error_ from climbing higher than permitted by i_max_/i_min_
		boost::tuple<double, double> bounds = boost::minmax<double>(i_min / i_gain, i_max / i_gain);
		i_error = boost::algorithm::clamp(i_error, bounds.get<0>(), bounds.get<1>());
	}

	// Calculate integral contribution to command
	i_term = i_gain * i_error;

	// Limit i_term so that the limit is meaningful in the output
	i_term = boost::algorithm::clamp(i_term, i_min, i_max);

	// Calculate derivative contribution to command
	d_term = d_gain * d_error;

	// Compute the command
	cmd = p_term + i_term + d_term;

	return cmd;
}

double PID::bound(double v, double b)
{
	if(v < -b)
		return -b;
	if(v > b)
		return b;
	return v;
}
