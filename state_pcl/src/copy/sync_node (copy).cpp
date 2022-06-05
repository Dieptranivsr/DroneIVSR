
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <random>
#include <nav_msgs/Odometry.h>
#include <queue>
#include <ros/ros.h>
#include <tuple>
#include <visualization_msgs/Marker.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <plan_env/raycast.h>

using namespace std;

class Syn{
public:
    Syn() {}
    ~Syn() {}

    void initSyn(ros::NodeHandle& nh);

    typedef std::shared_ptr<Syn> Ptr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    void depthPoseCb(const sensor_msgs::ImageConstPtr& img, const geometry_msgs::PoseStampedConstPtr& pose);
    void depthCallback(const sensor_msgs::ImageConstPtr& img);
    void poseCallback(const geometry_msgs::PoseStampedConstPtr& pose);

    ros::NodeHandle node;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped>
      SyncPolicyImagePose;
    typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;

    SynchronizerImagePose sync_image_pose;
    shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
    shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;

    ros::Subscriber depthImg, pose;
};

void Syn::initSyn(ros::NodeHandle& nh){
    node = nh;

    depth_sub_.reset( new message_filters::Subscriber<sensor_msgs::Image>(node, "/camera/depth/image_raw", 50));
    pose_sub_.reset( new message_filters::Subscriber<geometry_msgs::PoseStamped>(node, "/camera/pose", 25));

    sync_image_pose.reset(new message_filters::Synchronizer<SyncPolicyImagePose>(
        SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));
    sync_image_pose->registerCallback(boost::bind(&Syn::depthPoseCb, this, _1, _2));

    depthImg =
      node.subscribe<sensor_msgs::Image>("/camera/depth/image_rect_raw", 10, &Syn::depthCallback, this);
    pose =
      node.subscribe<geometry_msgs::PoseStamped>("/camera/pose", 10, &Syn::poseCallback, this);
     
}

void Syn::depthPoseCb(const sensor_msgs::ImageConstPtr& img, const geometry_msgs::PoseStampedConstPtr& pose){
    std::cout << "HIIIIIIIIIIIIIIIII" << std::endl;
}

void Syn::depthCallback(const sensor_msgs::ImageConstPtr& img) {

}

void Syn::poseCallback(const geometry_msgs::PoseStampedConstPtr& pose) {
    std::cout << "pose: " << pose->header.stamp << std::endl;

    if (!pose->pose.position.x)
        std::cout << "pose............ " << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "synchronizer");
    ros::NodeHandle node("~");
    

    std::cout << "Starting ...." << std::endl;
    Syn::Ptr syn_map;

    syn_map.reset(new Syn);
    syn_map->initSyn(node);

    ros::Duration(1.0).sleep();
    ros::spin();

    return 0;
}
