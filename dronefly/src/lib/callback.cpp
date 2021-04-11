#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/BatteryState.h>

#include <dronefly/callback.h>
/*
 * CALL FUNCTIONS
 */
// local pose callback
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
	current_pose = *msg;
}

// state callback
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

// battery status callback
void battery_cb(const sensor_msgs::BatteryState::ConstPtr& msg)
{
    current_batt = *msg;
}

// getlocalpose callback
void getlocalpose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	dest_pose = *msg;
}
