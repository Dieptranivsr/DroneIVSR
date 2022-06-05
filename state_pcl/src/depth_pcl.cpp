#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <iterator>

#include <Eigen/Dense>

using namespace std;

class DepthPcl{
public:
    DepthPcl() {}
    ~DepthPcl() {}

    void initDepthPcl(ros::NodeHandle& nh);

    cv::Mat depth_pic;

    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

    typedef std::shared_ptr<DepthPcl> Ptr;
private:
    void depthCallback(const sensor_msgs::ImageConstPtr& depth_msg);

    void stateCallback(const ros::TimerEvent& /*event*/);

    ros::NodeHandle node;

    ros::Subscriber depth_cb;
    ros::Publisher pointcloud_publisher;
    ros::Timer state_timer_;
};

void DepthPcl::initDepthPcl(ros::NodeHandle& nh){
    node = nh;

    depth_cb = node.subscribe<sensor_msgs::Image>("/depth", 50, &DepthPcl::depthCallback, this);

    pointcloud_publisher = node.advertise<sensor_msgs::PointCloud2>("/depthPointCloud", 10);

    state_timer_ = node.createTimer(ros::Duration(0.05), &DepthPcl::stateCallback, this);
}

void DepthPcl::depthCallback(const sensor_msgs::ImageConstPtr& depth_msg) {
    std::cout << "image: " << depth_msg->header.stamp << std::endl;

    cv_bridge::CvImagePtr depth_ptr;
    try
    {
        /* code */
        //cv::imshow("depth_view", cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1)->image);
        depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1); 

        //cv::waitKey();
    }
    catch(cv_bridge::Exception& e)
    {
        //ROS_ERROR("Could not convert from '%s' to 'mono16'.", depth_msg->encoding.c_str());
    }
    depth_pic = depth_ptr->image;
}

void DepthPcl::stateCallback(const ros::TimerEvent& /*event*/){
    sensor_msgs::PointCloud2 pub_pointcloud;
    PointCloud::Ptr cloud_msg (new PointCloud);

    // Use correct principal point from calibration
    const double camera_factor = 1;
    const double camera_cx = 320;
    const double camera_cy = 180;
    const double camera_fx = 320;
    const double camera_fy = 180;

    for (int v = 0; v < depth_pic.rows; ++v)
    {
        for (int u = 0; u < depth_pic.cols; ++u)
        {
            float d = depth_pic.ptr<float>(v)[u];

            if (d == 0)
                continue;

            pcl::PointXYZ pt;

            // Fill in XYZ
            pt.z = double(d) * camera_factor;
            pt.x = (u - camera_cx) * pt.z / camera_fx;
            pt.y = (v - camera_cy) * pt.z / camera_fy;

            cloud_msg->points.push_back(pt);
        }
    }

    cloud_msg->height = 1;
    cloud_msg->width = cloud_msg->points.size();
    cloud_msg->is_dense = false;

    pcl::toROSMsg(*cloud_msg, pub_pointcloud);
    pub_pointcloud.header.frame_id = "camera_depth_optical_frame";
    pub_pointcloud.header.stamp = ros::Time::now();

    pointcloud_publisher.publish(pub_pointcloud);

    cloud_msg->points.clear();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "node_state");
    ros::NodeHandle node("~");
    
    std::cout << "Starting ...." << std::endl;
    DepthPcl::Ptr depth_pcl;

    depth_pcl.reset(new DepthPcl);
    depth_pcl->initDepthPcl(node);

    ros::Duration(1.0).sleep();
    ros::spin();

    return 0;
}
