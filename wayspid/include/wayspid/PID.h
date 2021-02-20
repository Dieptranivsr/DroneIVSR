#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <geometry_msgs/QuaternionStamped.h>
#include <rosgraph_msgs/Clock.h>
#include <dynamic_reconfigure/server.h>
#include <wayspid/setPIDConfig.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <tf/transform_datatypes.h>

//using namespace geometry_msgs;
//using namespace std;
//using namespace ros;
//using namespace Eigen;

//using Eigen::Vector4d;

// Subscribers
ros::Subscriber odometry_sub;
ros::Subscriber trajectory_sub;
ros::Subscriber trajectory_velocity_sub;

// Publishers
ros::Publisher velocity_pub;

// Actual state
Eigen::Vector4d pose;
Eigen::Vector4d velocity;
Eigen::Vector4d pose_d;
Eigen::Vector4d velocity_d;

Eigen::Vector4d error;
Eigen::Vector4d error_old;
Eigen::Vector4d error_i;
Eigen::Vector4d error_d;

// Gains
double k_p_xy;
double k_i_xy;
double k_d_xy;
double k_p_z;
double k_i_z;
double k_d_z;
//double k_p;
//double k_i;
//double k_d;

bool new_odometry;

class PID{
        public:
          PID(int, char**);
          ~PID();
          void run();
        private:
          double denormalizeAngle(double a1, double a2);
};
