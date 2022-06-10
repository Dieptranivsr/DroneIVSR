#include <ros/ros.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>

#include <iostream>

using namespace std;

class LidarPcl{
public:
    LidarPcl() {}
    ~LidarPcl() {}

    void initLidarPcl(ros::NodeHandle& nh);

    // Refers to the conversion of pointers and non-pointers in pcl. 
    // Among them, new pcl::PointCloud<pcl::PointXYZ> is the data format of the original point cloud.
    typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

    typedef std::shared_ptr<LidarPcl> Ptr;
private:
    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& lidar_msg);

    ros::NodeHandle node;

    ros::Subscriber lidar_cb;
    ros::Publisher pointcloud_publisher;
};

void LidarPcl::initLidarPcl(ros::NodeHandle& nh){
    node = nh;

    lidar_cb = node.subscribe<sensor_msgs::PointCloud2>("/airsim_node/PX4/lidar/Lidar", 10, &LidarPcl::lidarCallback, this);

    pointcloud_publisher = node.advertise<sensor_msgs::PointCloud2>("/lidar_intensity", 10);
}

void LidarPcl::lidarCallback(const sensor_msgs::PointCloud2ConstPtr& lidar_msg) {
    std::cout << "lidar: " << lidar_msg->header.stamp << std::endl;

    // Declare the output point cloud format
    sensor_msgs::PointCloud2 pub_pointcloud;

    // When a function returns a pointer, there will often be unknowing errors. 
    // Instead of returning a pointer, you can get PointXYZ directly, and then convert it to Ptr
    PointCloud::Ptr cloud_int (new PointCloud); 

    // Get the field structure of this point cloud
    int pointBytes = lidar_msg->point_step;
    int offset_x;
    int offset_y;
    int offset_z;
    int offset_int;

    for ( int f=0; f<lidar_msg->fields.size(); ++f)
    {
        if( lidar_msg->fields[f].name == "x")
            offset_x = lidar_msg->fields[f].offset;
        if( lidar_msg->fields[f].name == "y")
            offset_y = lidar_msg->fields[f].offset;
        if( lidar_msg->fields[f].name == "z")
            offset_z = lidar_msg->fields[f].offset;
        if( lidar_msg->fields[f].name == "intensity")
            offset_int = lidar_msg->fields[f].offset;
    }

    // populate point cloud object
    for ( int p=0; p<lidar_msg->width; ++p)
    {
        pcl::PointXYZI newPoint;
        newPoint.x = *(float*)(&lidar_msg->data[0] + (pointBytes*p) + offset_x);
        newPoint.y = *(float*)(&lidar_msg->data[0] + (pointBytes*p) + offset_y);
        newPoint.z = *(float*)(&lidar_msg->data[0] + (pointBytes*p) + offset_z);
        newPoint.intensity = *(unsigned char*)(&lidar_msg->data[0] + (pointBytes*p) + offset_int);

        cloud_int->points.push_back(newPoint);
    }

    cloud_int->height   = 1;
    cloud_int->width    = cloud_int->points.size();
    cloud_int->is_dense = lidar_msg->is_dense;

    // Convert the pcl point cloud format to the point cloud format under ros, 
    // pcl::PointCloud<pcl::PointXYZI> to sensor_msgs::PointCloud2
    pcl::toROSMsg(*cloud_int, pub_pointcloud);

    // Assign the index of lidar_msg to pub_pointcloud.header here
    pub_pointcloud.header = lidar_msg->header;
    pointcloud_publisher.publish(pub_pointcloud);

    cloud_int->points.clear();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_pointcloud");
    ros::NodeHandle node("~");
    
    std::cout << "Starting to show full lidar message <pcl::PointCloud<pcl::PointXYZI>> ...." << std::endl;
    LidarPcl::Ptr syn_lidar;

    syn_lidar.reset(new LidarPcl);
    syn_lidar->initLidarPcl(node);

    ros::Duration(1.0).sleep();
    ros::spin();

    return 0;
}
