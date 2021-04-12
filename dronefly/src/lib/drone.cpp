#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/BatteryState.h>
#include <eigen_conversions/eigen_msg.h>

#include <dronefly/drone.h>

//check position
bool position_distance(geometry_msgs::PoseStamped e1, geometry_msgs::PoseStamped e2)
{
	Eigen::Vector3d ps1, ps2;
	tf::pointMsgToEigen(e1.pose.position, ps2);
	tf::pointMsgToEigen(e2.pose.position, ps1);

	double threshold = 0.05;       //0.01 m || 0.03(m) || 0.05m || 0.1 m
	ROS_DEBUG("Next setpoint: accepted error threshold: %1.3f", threshold);
	double distance = sqrt( (ps2 - ps1).x() * (ps2 - ps1).x() +
							(ps2 - ps1).y() * (ps2 - ps1).y() + 
							(ps2 - ps1).z() * (ps2 - ps1).z());
	std::cout << "[Distance] : " << distance << std::endl;
	
	if (distance <= threshold){
		return true;
	}
	else{
		return false;
	}
}

double _position_distance(geometry_msgs::PoseStamped e1, geometry_msgs::PoseStamped e2)
{
	Eigen::Vector3d ps1, ps2;
	tf::pointMsgToEigen(e1.pose.position, ps2);
	tf::pointMsgToEigen(e2.pose.position, ps1);

	double distance = sqrt( (ps2 - ps1).x() * (ps2 - ps1).x() +
							(ps2 - ps1).y() * (ps2 - ps1).y() +
							(ps2 - ps1).z() * (ps2 - ps1).z());
	std::cout << "[Distance] : " << distance << std::endl;

	return distance;
}
