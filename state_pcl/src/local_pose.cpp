// ROS core
#include <ros/ros.h>

#include "quadrotor_msgs/PositionCommand.h"
#include "visualization_msgs/Marker.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"

#include <iostream>

#include <Eigen/Dense>

using namespace std;

class VisualPose{
public:
    VisualPose() {}
    ~VisualPose() {}

    void initVisualPose(ros::NodeHandle& nh);

    typedef std::shared_ptr<VisualPose> Ptr;
private:
    void poseCallback(geometry_msgs::PoseStamped pose_msg);

    ros::NodeHandle node;

    ros::Subscriber pose_lc;
    ros::Publisher pose_w;

    visualization_msgs::Marker mk1, mk2;
};

void VisualPose::initVisualPose(ros::NodeHandle& nh){
    node = nh;

    pose_lc = node.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 50, &VisualPose::poseCallback, this);

    pose_w = node.advertise<geometry_msgs::PoseStamped>("/local/pose", 10);
}

void VisualPose::poseCallback(geometry_msgs::PoseStamped pose_msg) {
    std::cout << "pos in world: " << pose_msg.header.stamp << std::endl;

    geometry_msgs::PoseStamped pose_;
    pose_.header.frame_id = "world";
    pose_.pose.position.x = pose_msg.pose.position.x;
    pose_.pose.position.y = pose_msg.pose.position.y;
    pose_.pose.position.z = pose_msg.pose.position.z;
    pose_.pose.orientation.x = pose_msg.pose.orientation.x;
    pose_.pose.orientation.y = pose_msg.pose.orientation.y;
    pose_.pose.orientation.z = pose_msg.pose.orientation.z;
    pose_.pose.orientation.w = pose_msg.pose.orientation.w;

    pose_w.publish(pose_);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visual_pose");
    ros::NodeHandle node("~");
    
    std::cout << "Starting to show pose of flight outdoor ...." << std::endl;
    VisualPose::Ptr visualPose;

    visualPose.reset(new VisualPose);
    visualPose->initVisualPose(node);

    ros::Duration(1.0).sleep();
    ros::spin();

    return 0;
}
