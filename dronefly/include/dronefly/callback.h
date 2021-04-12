#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/PoseStamped.h>

// the setpoint publishing rate MUST be faster than 2Hz
int rate = 20;

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

// getlocalpose callback
geometry_msgs::PoseStamped dest_pose;
void getlocalpose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
