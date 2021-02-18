#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <rosgraph_msgs/Clock.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/NavSatFix.h>
#include <Eigen/Dense>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Thrust.h>
#include <mavros_msgs/RCIn.h>
#include <dynamic_reconfigure/server.h>
#include <wayspid/setSafeD9Config.h>
#include <eigen_conversions/eigen_msg.h>
#include <sensor_msgs/BatteryState.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>

/*
#define MAX_X   10
#define MAX_Y   4
#define MAX_Z   5
#define MAX_V   3
//#define MAX_RP  (M_PI / 10)
#define MAX_RP  (M_PI / 2)
*/
// Fast flight
#define MAX_X   110
#define MAX_Y   50
#define MAX_Z   10
#define MAX_V   3
//#define MAX_RP  (M_PI / 10)
#define MAX_RP  (M_PI)


//using namespace geometry_msgs;
//using namespace std;
//using namespace ros;
//using namespace Eigen;

//using Eigen::Vector4d;

// Publishers
ros::Publisher reset_pub;
ros::Publisher takeoff_pub;
ros::Publisher land_pub;
ros::Publisher velocity_pub;
ros::Publisher odometry_pub;
ros::Publisher noisy_odometry_pub;
ros::Publisher attitude_pub;
ros::Publisher throttle_pub;
ros::Publisher position_pub;
ros::Publisher test_msg_pub;
ros::Publisher att_com_pub;
ros::Publisher att_local_pub;
ros::Publisher raw_att_com_pub;
ros::Publisher raw_pos_com_pub;

// Subsribers
ros::Subscriber command_sub;
ros::Subscriber command_position_sub;
ros::Subscriber command_velocity_sub;
ros::Subscriber command_attitude_sub;
ros::Subscriber optitrack_sub;
ros::Subscriber noise_sub;
ros::Subscriber battery_sub;
ros::Subscriber state_sub;
ros::Subscriber velocity_actual_sub;
ros::Subscriber local_pos_sub;
ros::Subscriber RC_input_sub;

// Services
ros::ServiceClient set_mode_client;
ros::ServiceClient arming_client;
ros::ServiceClient land_client;

//MavROS
mavros_msgs::State current_state;
mavros_msgs::CommandBool arm_cmd;
mavros_msgs::SetMode offb_set_mode;
mavros_msgs::CommandTOL land_cmd;
mavros_msgs::RCIn RCinputs;

bool RCavailable =false;

Eigen::Vector4d position_d;
Eigen::Vector4d velocity_d;
Eigen::Vector4d attitude_d;

// Variables
geometry_msgs::Point position;
geometry_msgs::Point orientation;
std::vector<geometry_msgs::Twist> velocities;
nav_msgs::Odometry noise;
ros::Time time_old;
int sequence;
double yaw;
bool stop;
int controller;
double thrust_multiplier;

double error = 0.0, error_prev = 0.0;

class SafeD9{
	public:
		SafeD9(int, char**);
		~SafeD9();
		void run();
    private:
		void push(double vx, double vy, double vz, double p, double q, double r);
		geometry_msgs::Twist filter();
};
