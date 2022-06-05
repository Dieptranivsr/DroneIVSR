#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <iostream>

#include <state_pcl/DroneState.h>

#include <Eigen/Dense>

using namespace std;

class Syn{
public:
    Syn() {}
    ~Syn() {}

    void initSyn(ros::NodeHandle& nh);

    Eigen::Vector3d pos_state;
    Eigen::Vector4d ori_state;
    Eigen::Vector3d vel_linear_state;
    Eigen::Vector3d vel_angle_state;

    typedef std::shared_ptr<Syn> Ptr;
private:
    void poseCallback(const geometry_msgs::PoseStampedConstPtr& pos_);
    void velocityLocalCallback(const geometry_msgs::TwistStampedConstPtr& vel_);
    void stateCallback(const ros::TimerEvent& /*event*/);

    ros::NodeHandle node;

    ros::Subscriber localPose, localVel;
    ros::Publisher state_pub_;
    ros::Timer state_timer_;
};

void Syn::initSyn(ros::NodeHandle& nh){
    node = nh;

    localPose = node.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &Syn::poseCallback, this);
    localVel = node.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 10, &Syn::velocityLocalCallback, this);

    state_pub_ = node.advertise<state_pcl::DroneState>("/state", 10);

    state_timer_ = node.createTimer(ros::Duration(0.05), &Syn::stateCallback, this);
}

void Syn::poseCallback(const geometry_msgs::PoseStampedConstPtr& pos_) {
    std::cout << "pose: " << pos_->header.stamp << std::endl;

	pos_state << pos_->pose.position.x, pos_->pose.position.y, pos_->pose.position.z;
    ori_state << pos_->pose.orientation.x, pos_->pose.orientation.y, pos_->pose.orientation.z, pos_->pose.orientation.w;
}

void Syn::velocityLocalCallback(const geometry_msgs::TwistStampedConstPtr& vel_) {
    std::cout << "vel: " << vel_->header.stamp << std::endl;
 
    vel_linear_state << vel_->twist.linear.x, vel_->twist.linear.y, vel_->twist.linear.z;
    vel_angle_state << vel_->twist.angular.x, vel_->twist.angular.y, vel_->twist.angular.z;
}

void Syn::stateCallback(const ros::TimerEvent& /*event*/){
    state_pcl::DroneState pubData;
    
    pubData.header.frame_id = "world";
    pubData.header.stamp = ros::Time::now();
    pubData.pose.pose.position.x = pos_state(0);
    pubData.pose.pose.position.y = pos_state(1);
    pubData.pose.pose.position.z = pos_state(2);
    pubData.pose.pose.orientation.x = ori_state(0);
    pubData.pose.pose.orientation.y = ori_state(1);
    pubData.pose.pose.orientation.z = ori_state(2);
    pubData.pose.pose.orientation.w = ori_state(3);

    pubData.twist.linear.x = vel_linear_state(0);
    pubData.twist.linear.y = vel_linear_state(1);
    pubData.twist.linear.z = vel_linear_state(2);
    pubData.twist.angular.x = vel_angle_state(0);
    pubData.twist.angular.y = vel_angle_state(1);
    pubData.twist.angular.z = vel_angle_state(2);    

    state_pub_.publish(pubData);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "node_state");
    ros::NodeHandle node("~");
    
    std::cout << "Starting ...." << std::endl;
    Syn::Ptr syn_map;

    syn_map.reset(new Syn);
    syn_map->initSyn(node);

    ros::Duration(1.0).sleep();
    ros::spin();

    return 0;
}
