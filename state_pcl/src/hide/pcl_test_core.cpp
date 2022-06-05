#include "pcl_test_core.h"

PclTestCore::PclTestCore(ros::NodeHandle &nh){
    sub_point_cloud_ = nh.subscribe("/velodyne_points",10, &PclTestCore::point_cb, this);//The prviate of sub_point_cloud here in pcl_test_core.h
    
    pub_filtered_points_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 10);
    
    ros::spin();//Callback function
}

PclTestCore::~PclTestCore(){}

void PclTestCore::Spin(){
    
}
//It can be seen that the above three steps echo pcl_test_node.cpp, yeah!!!

void PclTestCore::point_cb(const sensor_msgs::PointCloud2ConstPtr & in_cloud_ptr){
    //Define two point cloud pointers, declare the point cloud format for storing the original data and the filtered data
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    
    //pcl::PointCloud<pcl::PointXYZ>::Ptr (disconnected here) cloud_Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    //Refers to the conversion of pointers and non-pointers in pcl. Among them, new pcl::PointCloud<pcl::PointXYZ> is the data format of the original point cloud.
    //When a function returns a pointer, there will often be unknowing errors. Instead of returning a pointer, you can get PointXYZ directly, and then convert it to Ptr.
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    
    //new pcl::PointCloud<pcl::PointXYZI>, store the filtered data format.
    pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);
    
    //ros is converted to the point cloud data format in PCL, sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXYZ> 
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    //Instantiate filtering, vg is the abbreviation of voxelgrid, replace all of the previous ones, to provide convenience for subsequent programming, equivalent to import numpy as np np.xxx

    vg.setInputCloud(current_pc_ptr);//Set the input filter
    vg.setLeafSize(0.2f, 0.2f, 0.2f);//Set the voxel grid size
    vg.filter(*filtered_pc_ptr);//store the filtered point cloud
    
    //Then convert the filtered point cloud into the data format under ros and publish it.
    sensor_msgs::PointCloud2 pub_pc;//Declare the output point cloud format.
    pcl::toROSMsg(*filtered_pc_ptr, pub_pc);//Convert the pcl point cloud format to the point cloud format under ros, pcl::PointCloud<pcl::PointXYZ> to sensor_msgs::PointCloud2. The first parameter is the filtered point cloud in pcl xyz format, and the second parameter is the ros pointcloud2 format after the capture, and the point cloud will be published after publishing.
    
    pub_pc.header = in_cloud_ptr->header; 
    //Assign the index of in_cloude_ptr to pub_pc.header here? ? ? This was not found.

    pub_filtered_points_.publish(pub_pc);
    //pub_filtered_points_ is the previously established publishing node, and assign a value to the node here.
}


//Refer to sitwangmin blogger from here, thanks!
//Author: sitwangmin 
//Source: CSDN 
//Original: https://blog.csdn.net/u010284636/article/details/79214841 

//ROS to PCL data format
//sensor_msgs::PointCloud2 to pcl::PCLPointCloud2 
//pcl_conversions::toPCL(sensor_msgs::PointCloud2, pcl::PCLPointCloud2)

//sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXYZ> 
//pcl::fromROSMsg (sensor_msgs::PointCloud2, pcl::PointCloud<pcl::PointXYZ>);

//PCL to ROS data
//pcl::PCLPointCloud2 to sensor_msgs::PointCloud2
//pcl_conversions::fromPCL(pcl::PCLPointCloud2, sensor_msgs::PointCloud2);

//pcl::PointCloud<pcl::PointXYZ> to sensor_msgs::PointCloud2
//pcl::toROSMsg (pcl::PointCloud<pcl::PointXYZ>,sensor_msgs::PointCloud2);

//Data conversion in PCL 
//pcl::PCLPointCloud2 to pcl::PointCloud <pcl::PointXYZ>
//pcl::fromPCLPointCloud2(pcl::PCLPointCloud2,pcl::PointCloud<pcl::PointXYZ>);

//pcl::PointCloud<pcl::PointXYZ> to pcl::PCLPointCloud2
//pcl::toPCLPointCloud2(pcl::PointCloud<pcl::PointXYZ>, pcl::PCLPointCloud2);
