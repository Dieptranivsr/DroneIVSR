//#pragma once

#include <array>
#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>

#include <dronefly/compute_pid.h>

namespace pidcontroller{
class PIDController
{
public:
	PIDController();
	~PIDController();

	double MAX_vel = 1;      // 0.5  1  2  3 (m/s)

	// /*
	//  * PID Setup
	//  */
	// Sets up the PID values for computation of the linear velocities effort
	void setup_linvel_pid(double p_gain, double i_gain, double d_gain, double i_max, double i_min);
	Eigen::Vector3d compute_linvel_effort(Eigen::Vector3d goal, Eigen::Vector3d current, ros::Time last_time);

private:
	computepid::PID pid_linvel_x;
	computepid::PID pid_linvel_y;
	computepid::PID pid_linvel_z;
};
}
