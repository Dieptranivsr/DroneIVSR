#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>

#include <iostream>
#include <Eigen/Dense>

using namespace std;

class Syn{
public:
    Syn() {}
    ~Syn() {}

    void initSyn(ros::NodeHandle& nh);

    typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

    typedef std::shared_ptr<Syn> Ptr;
private:
    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& lidar_msg);

    ros::NodeHandle node;

    ros::Subscriber lidar_cb;
    ros::Publisher pointcloud_publisher;
};

void Syn::initSyn(ros::NodeHandle& nh){
    node = nh;

    lidar_cb = node.subscribe<sensor_msgs::PointCloud2>("/airsim_node/PX4/lidar/Lidar", 10, &Syn::lidarCallback, this);

    pointcloud_publisher = node.advertise<sensor_msgs::PointCloud2>("/lidar_intensity", 10);
}

void Syn::lidarCallback(const sensor_msgs::PointCloud2ConstPtr& lidar_msg) {
    std::cout << "lidar: " << lidar_msg->header.stamp << std::endl;

    PointCloud::Ptr current_pc_ptr (new PointCloud); 

    PointCloud::Ptr filtered_pc_ptr (new PointCloud); 
    
    pcl::fromROSMsg(*lidar_msg, *current_pc_ptr);

    pcl::VoxelGrid<pcl::PointXYZI> vg;

    vg.setInputCloud(current_pc_ptr);   //Set the input filter
    vg.setLeafSize(0.2f, 0.2f, 0.2f);   //Set the voxel grid size
    vg.filter(*filtered_pc_ptr);        //store the filtered point cloud

    sensor_msgs::PointCloud2 laserCloud;;
    pcl::toROSMsg(*filtered_pc_ptr, laserCloud);

    laserCloud.header = lidar_msg->header;
    pointcloud_publisher.publish(laserCloud);
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
