#include <iostream>
#include <array>
#include <random>

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <angles/angles.h>

/*
 * callbacks
 */
//local pose callback
geometry_msgs::PoseStamped current_pose;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);

//state callback
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg);

// battery status callback
sensor_msgs::BatteryState current_batt;
void battery_cb(const sensor_msgs::BatteryState::ConstPtr& msg);

/*
 * Gaussian noise generator for accepted position threshold
 */
std::array<double, 100> threshold;
std::array<double, 100> threshold_definition();

bool position_distance(geometry_msgs::PoseStamped e1, geometry_msgs::PoseStamped e2);
double _position_distance(geometry_msgs::PoseStamped e1, geometry_msgs::PoseStamped e2);

Eigen::Vector3d pos_setpoint(int tr_x, int tr_y, int tr_z);
Eigen::Vector3d circle_shape(int angle);
Eigen::Vector3d eight_shape(int angle);
Eigen::Vector3d ellipse_shape(int angle);
