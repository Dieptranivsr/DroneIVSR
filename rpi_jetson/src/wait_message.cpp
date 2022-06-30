#include "ros/ros.h"
#include "std_msgs/String.h"
#include <boost/shared_ptr.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wait_message");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        boost::shared_ptr<std_msgs::String const> msg;
        msg = ros::topic::waitForMessage<std_msgs::String>("custom_topic", ros::Duration(5));

        if (msg)
        {
            ROS_INFO("%s", msg->data.c_str());
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
