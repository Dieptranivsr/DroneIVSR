#include <iostream>
#include <stdio.h>
#include <vector>

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/BatteryState.h>

double _timer; //travel time - fly from A to B
/****** DEFINE CONSTANTS ******/
#define td_PI 3.141592653589793238462643383279502884L

// convert angle
double degree(double _rad);
double radian(double _deg);

/*
 * CALL FUNCTIONS
 */

//state callback
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg);

//local pose callback
geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);

// battery status callback
sensor_msgs::BatteryState current_batt;
void battery_cb(const sensor_msgs::BatteryState::ConstPtr& msg);

double distance(geometry_msgs::PoseStamped e1, geometry_msgs::PoseStamped e2);
bool position_distance(geometry_msgs::PoseStamped e1, geometry_msgs::PoseStamped e2);
geometry_msgs::Twist compute_velocity(geometry_msgs::PoseStamped e1, geometry_msgs::PoseStamped e2, double timer1);

// check current pose
//void check_current(geometry_msgs::PoseStamped _pose, sensor_msgs::BatteryState _battery);