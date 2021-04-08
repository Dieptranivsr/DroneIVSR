#include <iostream>
#include <array>
#include <random>

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <eigen3/Eigen/Dense>
/*
 * PID Controllers (from line 11th to 46)
 */
bool valid_p_error_last_x = false; 	/**< Is saved position state valid for derivative state calculation */
bool valid_p_error_last_y = false;
bool valid_p_error_last_z = false;
double p_error_last_x = 0.0; 		/**< Save position state for derivative state calculation. */
double p_error_last_y = 0.0;
double p_error_last_z = 0.0;
double p_error_x = 0.0; 				/**< Position error. */
double p_error_y = 0.0;
double p_error_z = 0.0;
double i_error_x = 0.0; 				/**< Integral of position error. */
double i_error_y = 0.0;
double i_error_z = 0.0;
double d_error_x = 0.0; 				/**< Derivative of position error. */
double d_error_y = 0.0;
double d_error_z = 0.0;
double cmd_x = 0.0;     			/**< Command to send. */
double cmd_y = 0.0;
double cmd_z = 0.0;

double p_gain_ = 0.0;
double i_gain_ = 0.0;
double d_gain_ = 0.0;
double i_max_ = 0.0;
double i_min_ = 0.0;

void setup_livel_pid(double p_gain, double i_gain, double d_gain, double i_max, double i_min);
double computeCommand_x(double error, ros::Duration dt);
double computeCommand_y(double error, ros::Duration dt);
double computeCommand_z(double error, ros::Duration dt);

double computeCommandX(double error, double error_dot, ros::Duration dt);
double computeCommandY(double error, double error_dot, ros::Duration dt);
double computeCommandZ(double error, double error_dot, ros::Duration dt);
Eigen::Vector3d compute_linvel_effort(Eigen::Vector3d goal, Eigen::Vector3d current, ros::Time last_time);
//Eigen::Vector3d compute_linvel_effort(geometry_msgs::PoseStamped goal, geometry_msgs::PoseStamped current, ros::Time last_time);

#define MAX_vel 0.5         // 3 // 1
float bound(float v, float b);

//Eigen::Vector3d _goal, _current;
