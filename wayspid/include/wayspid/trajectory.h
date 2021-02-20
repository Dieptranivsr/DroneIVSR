#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <rosgraph_msgs/Clock.h>
#include <dynamic_reconfigure/server.h>
#include <wayspid/setTrajectoryConfig.h>
#include <Eigen/Dense>
#include <ros/package.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>

// Subscriber
ros::Subscriber attitude_sub;
ros::Subscriber command_sub;
ros::Subscriber local_pos_sub;
	
// Pushlisher
ros::Publisher trajectory_pub;
ros::Publisher velocity_pub;
ros::Publisher trajectory_type_pub;

std_msgs::Int32 traj_type;

// Pose
Eigen::Vector4d pose_d;
Eigen::Vector4d pose_act;

// Trajectory type
int trajectory_type;

// Trajectory speed
double speed;
double straight_speed;

// Yaw angle
double yaw_d, initial_local_yaw, last_yaw;

// Waypoints
Eigen::MatrixXd waypoints;
int waypoint;

// Time
double t;
double t_straight;

class Trajectory{
public:
	Trajectory(int, char**);
	~Trajectory();
	
	void run();
private:
	double denormalizeAngle(double deA);
	double distance(Eigen::Vector3d vec1, Eigen::Vector3d vec2);
};