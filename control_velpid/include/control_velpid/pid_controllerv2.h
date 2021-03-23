#include <iostream>
#include <array>
#include <random>

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <eigen3/Eigen/Dense>


/*
 * Version2
 */

//Eigen::Vector4d pose_goal(5.0, 5.0, 2.5, -91*M_PI/180);
Eigen::Vector4d pose_goal(0, 5.0, 3, -91*M_PI/180);

Eigen::Vector3d Kp(0.4, 0.4, 0.2);
Eigen::Vector3d Ki(0.0, 0.0, 0.0);
Eigen::Vector3d Kd(0.2, 0.2, 0.2);
double yaw_gain = 0.5;

ros::Duration _dt;
Eigen::Vector3d _goal, _current;
Eigen::Vector3d distance_error, distance_integral(0, 0, 0), distance_derivative;
Eigen::Vector3d previous_distance_error(0, 0, 0);
Eigen::Vector3d vel_world, vel_body;
Eigen::Vector3d Rbv1, Rbv2, Rbv3;
double roll, pitch, yaw;
double yaw_rate_world;

double quaternion_to_euler(double x, double y, double z, double w);
double compute_linvel_effort_v2(geometry_msgs::PoseStamped goal, geometry_msgs::PoseStamped current, ros::Time last_time);
