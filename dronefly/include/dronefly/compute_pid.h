#pragma once

#include <iostream>
#include <boost/algorithm/clamp.hpp>
#include <boost/algorithm/minmax.hpp>

#include <ros/ros.h>

namespace computepid {
class PID{
private:
    double p_gain;   /**< Proportional gain. */
    double i_gain;   /**< Integral gain. */
    double d_gain;   /**< Derivative gain. */
    double i_max;    /**< Maximum allowable integral term. */
    double i_min;    /**< Minimum allowable integral term. */

public:
    bool valid_p_error_last = false;	// *** Is saved position state valid for derivative state calculation ***
    double p_error_last = 0.0;			// *** Save position state for derivative state calculation
    double p_error = 0.0;				// *** Position error
    double i_error = 0.0;				// *** Integral of position error
    double d_error = 0.0;				// *** Derivative of position error
    double cmd = 0.0;					// *** Command to send

	PID(double p=0.0, double i=0.0, double d=0.0, double imax=0.0, double imin=-0.0);
	~PID();

	void initPID(double p, double i, double d, double imax, double imin);

	double computeCommand(double error, ros::Duration dt);
	double computeCommand(double error, double error_dot, ros::Duration dt);

	double bound(double v, double b);

};
};	// namespace pidcontroller
