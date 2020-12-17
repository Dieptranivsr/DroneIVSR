#include <iostream>
#include <array>
#include <random>

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>

/*
 * callbacks
 */
geometry_msgs::PoseStamped localpos;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg);

/*
 * Gaussian noise generator for accepted position threshold
 */
std::array<double, 100> threshold;
std::array<double, 100> threshold_definition();

bool position_distance(geometry_msgs::PoseStamped e1, geometry_msgs::PoseStamped e2);
Eigen::Vector3d pos_setpoint(int tr_x, int tr_y, int tr_z);
Eigen::Vector3d circle_shape(int angle);


