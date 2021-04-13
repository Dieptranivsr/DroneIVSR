#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

//check position
bool position_distance(geometry_msgs::PoseStamped e1, geometry_msgs::PoseStamped e2);
double _position_distance(geometry_msgs::PoseStamped e1, geometry_msgs::PoseStamped e2, bool show);

