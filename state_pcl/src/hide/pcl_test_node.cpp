#include "pcl_test_core.h"

int main(int argc, char **argv)//main function, node entry
{
    ros::init(argc, argv, "pcl_test");//Initialize the node, the third parameter node_name, node parameter
    ros::NodeHandle nh;//nh each node corresponds to a handle, instantiate the node?
    PclTestCore core(nh);//Didn't find it, it is related to the subsequent node startup anyway

    return 0;
}