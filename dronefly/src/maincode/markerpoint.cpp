#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <visualization_msgs/Marker.h>

#include <dronefly/compute_pid.h>
#include <dronefly/callback.h>


int main( int argc, char **argv)
{
	//
	// ROS INITIALIZATION
	//
	ros::init(argc, argv, "markerpoints");
	ros::NodeHandle td;

	ros::Subscriber getlocalpose_sub = td.subscribe<geometry_msgs::PoseStamped>
			("/drone/get/position_local", 10, getlocalpose_cb);
	ros::Publisher marker_pub = td.advertise<visualization_msgs::Marker>
			("visualization_marker", 10);

	// the setpoint publishing rate MUST be faster than 2Hz
	int rate = 20;
	ros::Rate loop_rate(rate);


    visualization_msgs::Marker points, line_strip, landmark;
    points.header.frame_id = line_strip.header.frame_id = landmark.header.frame_id = "map";
    points.header.stamp = line_strip.header.stamp = landmark.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = landmark.ns = "points_and_lines";
    points.action = line_strip.action = landmark.action = visualization_msgs::Marker::ADD;
    points.id = 0;
    line_strip.id = 1;
    landmark.id = 2;
    points.type = visualization_msgs::Marker::SPHERE_LIST;         // POINTS, SPHERE_LIST
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    landmark.type = visualization_msgs::Marker::POINTS;
    points.pose.orientation.w = line_strip.pose.orientation.w = landmark.pose.orientation.w = 1.0;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.05;         // 0.05, 0.1
    points.scale.y = 0.05;
    points.scale.z = 0.05;

    line_strip.scale.x = 0.03;
    line_strip.scale.x = 0.03;
    line_strip.scale.x = 0.03;

    landmark.scale.x = 0.1;
    landmark.scale.y = 0.1;
    landmark.scale.z = 0.1;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // Landmark is red
    landmark.color.r = 1.0;
    landmark.color.a = 1.0;

	while( ros::ok())
	{
		points.points.push_back(current_pose.pose.position);
		line_strip.points.push_back(current_pose.pose.position);
		marker_pub.publish(points);
		marker_pub.publish(line_strip);

		landmark.points.push_back(dest_pose.pose.position);
		marker_pub.publish(dest_pose);
	}

	return 0;
}
