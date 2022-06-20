// ROS core
#include <ros/ros.h>

#include "quadrotor_msgs/PositionCommand.h"
#include "visualization_msgs/Marker.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"

#include <iostream>

#include <Eigen/Dense>

using namespace std;

class VisualTraj{
public:
    VisualTraj() {}
    ~VisualTraj() {}

    void initVisualTraj(ros::NodeHandle& nh);

    typedef std::shared_ptr<VisualTraj> Ptr;
private:
    void trajCallback(quadrotor_msgs::PositionCommand pos_msg);
    void odomCallback(nav_msgs::Odometry odom_msg);

    ros::NodeHandle node;

    ros::Subscriber traj_cb, odom_cb;
    ros::Publisher traj_pub, pose_pub, odom_pub;

    visualization_msgs::Marker mk1, mk2;
};

void VisualTraj::initVisualTraj(ros::NodeHandle& nh){
    node = nh;

    traj_cb = node.subscribe<quadrotor_msgs::PositionCommand>("/planning/ref_traj", 50, &VisualTraj::trajCallback, this);
    odom_cb = node.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 50, &VisualTraj::odomCallback, this);

    traj_pub = node.advertise<visualization_msgs::Marker>("/trajectory/points", 10);
    pose_pub = node.advertise<visualization_msgs::Marker>("/trajectory/pose", 10);
    odom_pub = node.advertise<geometry_msgs::PoseStamped>("/local/odom", 10);

    mk1.header.frame_id = mk2.header.frame_id = "world";
    mk1.header.stamp = mk2.header.stamp = ros::Time::now();
    mk1.type = mk2.type = visualization_msgs::Marker::SPHERE_LIST;
    mk1.action = mk2.action = visualization_msgs::Marker::ADD;
}

void VisualTraj::trajCallback(quadrotor_msgs::PositionCommand pos_msg) {
    std::cout << "Trajectory: " << pos_msg.header.stamp << std::endl;

    mk1.pose.orientation.x = 0.0;
    mk1.pose.orientation.y = 0.0;
    mk1.pose.orientation.z = 0.0;
    mk1.pose.orientation.w = 1.0;

    mk1.color.r = 0.15;
    mk1.color.g = 1.0;
    mk1.color.b = 0.8;
    mk1.color.a = 0.8;

    mk1.scale.x = 0.05;
    mk1.scale.y = 0.05;
    mk1.scale.z = 0.05;

    geometry_msgs::Point pt;
    pt.x = pos_msg.position.x;
    pt.y = pos_msg.position.y;
    pt.z = pos_msg.position.z;
    mk1.points.push_back(pt);
    
    traj_pub.publish(mk1);
}

void VisualTraj::odomCallback(nav_msgs::Odometry odom_msg) {
    std::cout << "Odometry: " << odom_msg.header.stamp << std::endl;

    geometry_msgs::PoseStamped odom;
    odom.header.frame_id = "world";
    odom.pose.position.x = odom_msg.pose.pose.position.x;
    odom.pose.position.y = odom_msg.pose.pose.position.y;
    odom.pose.position.z = odom_msg.pose.pose.position.z;
    odom.pose.orientation.x = odom_msg.pose.pose.orientation.x;
    odom.pose.orientation.y = odom_msg.pose.pose.orientation.y;
    odom.pose.orientation.z = odom_msg.pose.pose.orientation.z;
    odom.pose.orientation.w = odom_msg.pose.pose.orientation.w;

    odom_pub.publish(odom);

    mk2.pose.orientation.x = 0.0;
    mk2.pose.orientation.y = 0.0;
    mk2.pose.orientation.z = 0.0;
    mk2.pose.orientation.w = 1.0;

    mk2.color.r = 1.0;
    mk2.color.g = 0.15;
    mk2.color.b = 0.9;
    mk2.color.a = 0.8;

    mk2.scale.x = 0.05;
    mk2.scale.y = 0.05;
    mk2.scale.z = 0.05;

    geometry_msgs::Point pt;
    pt.x = odom_msg.pose.pose.position.x;
    pt.y = odom_msg.pose.pose.position.y;
    pt.z = odom_msg.pose.pose.position.z;
    mk2.points.push_back(pt);
    
    pose_pub.publish(mk2);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visual_trajectory");
    ros::NodeHandle node("~");
    
    std::cout << "Starting to show trajectory of flight outdoor ...." << std::endl;
    VisualTraj::Ptr visualTraj;

    visualTraj.reset(new VisualTraj);
    visualTraj->initVisualTraj(node);

    ros::Duration(1.0).sleep();
    ros::spin();

    return 0;
}
