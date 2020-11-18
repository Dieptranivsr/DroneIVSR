#include <ros/ros.h>
#include <iostream>
#include <string>
#include <stdlib.h>                    /* <abs> */

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <geometry_msgs/Point.h>
#include <mavros_msgs/SetMavFrame.h>
#include <geometry_msgs/TwistStamped.h>
#include <eigen_conversions/eigen_msg.h>

#include <test_velocity/test_lib.hpp>
#include <test_velocity/pbPlots.hpp>
#include <test_velocity/supportLib.hpp>
#include <array>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <math.h>

namespace testsetup {
/**
 * @brief Offboard controller testter
 * 
 * Test offboard velocity control
 */
class OffboardControl{
public:
	OffboardControl():		
	{ };
	
	void init(){
		/**
		 * @brief Setup of the test conditions
		 */
		
	}
	
	/**
	 * @brief Gaussian noise generator for accepted position threshold
	 */
	std::array<double, 100> threshold_definition(){
		std::random_device rd;
		std::mt19937 gen(rd());
		std::array<double, 100> th_values;

		std::normal_distribution<double> th(0.1f,0.05f);

		for (auto &value : th_values) {
			value = th(gen);
		}
		return th_values;
	}
	
	/* -*- callbacks -*- */
	
	//state callback
	mavros_msgs::State current_state;
	void state_cb(const mavros_msgs::State::ConstPtr& msg)
	{
		current_state = *msg;
	}

	//local pose callback
	geometry_msgs::PoseStamped current_pose;
	void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
	{
		current_pose = *msg;
	}
};
};	// namespace testsetup