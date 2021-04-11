#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <dronefly/compute_pid.h>
#include <dronefly/pid.h>

using namespace pidcontroller;

PIDController::PIDController()
{};
PIDController::~PIDController()
{};

void PIDController::setup_linvel_pid(double p_gain, double i_gain, double d_gain, double i_max, double i_min){
	// PID values
	std::array<double, 3> linvel_pid = { {p_gain, i_gain, d_gain} };

	// Min/max bounds for the integral windup
	double linvel_imin = i_min;
	double linvel_imax = i_max;

	pid_linvel_x.initPID(linvel_pid[0], linvel_pid[1], linvel_pid[2], linvel_imax, linvel_imin);
	pid_linvel_y.initPID(linvel_pid[0], linvel_pid[1], linvel_pid[2], linvel_imax, linvel_imin);
	pid_linvel_z.initPID(linvel_pid[0], linvel_pid[1], linvel_pid[2], linvel_imax, linvel_imin);
}

Eigen::Vector3d PIDController::compute_linvel_effort(Eigen::Vector3d goal, Eigen::Vector3d current, ros::Time last_time){
	double lin_vel_x = pid_linvel_x.computeCommand(goal.x() - current.x(), ros::Time::now() - last_time);
	double lin_vel_y = pid_linvel_y.computeCommand(goal.y() - current.y(), ros::Time::now() - last_time);
	double lin_vel_z = pid_linvel_z.computeCommand(goal.z() - current.z(), ros::Time::now() - last_time);

	lin_vel_x = pid_linvel_x.bound(lin_vel_x, MAX_vel);
	lin_vel_y = pid_linvel_y.bound(lin_vel_y, MAX_vel);
	lin_vel_z = pid_linvel_x.bound(lin_vel_z, MAX_vel);

	return Eigen::Vector3d(lin_vel_x, lin_vel_y, lin_vel_z);
}
