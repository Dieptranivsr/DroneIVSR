#pragma once

#include <ros/ros.h>      
//Import ROS system including core public header files

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>   
//Point type header file
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>  

#include <pcl/filters/voxel_grid.h>

#include <sensor_msgs/PointCloud2.h>
//These .h files are a series of template files, similar to library files such as numpy and matplotlib in python, which can be directly called to simplify programming. But what each library does is not found.

class PclTestCore
{
private:
    ros::Subscriber sub_point_cloud_;//Create a subscription node for receiving point cloud information
    ros::Publisher pub_filtered_points_;//Created a node for publishing filtering
    
    void point_cb(const sensor_msgs::PointCloud2ConstPtr& in_cloud);
    //void point_cb is to declare a function, which sets a data type of sensor_msgs::PointCloud2ConstPtr& in_cloud parameter, and const modifies the parameters in the function here. Convert point cloud format sensor_mgs/pointcloud2 to pcl/pointcloud

public:
    PclTestCore(ros::NodeHandle &nh);
    ~PclTestCore();
    void Spin();
};

//1, public modified member variables
//It can be accessed anywhere in the program, which is the meaning of a public variable, which can be directly accessed by an instance of the class without passing a member function
//2, private modified member variables
//Only directly accessible within the class, private, the instance of the class can only be accessed through member functions, this can be used to hide