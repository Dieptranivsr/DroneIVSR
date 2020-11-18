#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Float64.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_velocity");
	
	ros::NodeHandle td;
	
	ros::Publisher chatter_pub = td.advertise<std_msgs::Float64>("chatter", 1000);
	
	ros::Rate loop_rate(10);
	
	int count = 0;
	while(ros::ok())
	{
		std_msgs::Float64 msg;
		
		msg.data = count;

		chatter_pub.publish(msg);
		
		ros::spinOnce();
		
		loop_rate.sleep();
		++count;
	}
	
	return 0;
}
