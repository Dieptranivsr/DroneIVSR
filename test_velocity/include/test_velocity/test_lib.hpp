/*
 * #include <test_velocity/test_lib.hpp>
 */
#include <iostream>
#include <stdio.h>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

void captureGraph(std::vector<double> _xs, std::vector<double> _ys, std::string abc);
std::string getName();

double distance(geometry_msgs::PoseStamped e1, geometry_msgs::PoseStamped e2);
bool position_distance(geometry_msgs::PoseStamped e1, geometry_msgs::PoseStamped e2);
geometry_msgs::Twist compute_velocity(geometry_msgs::PoseStamped e1, geometry_msgs::PoseStamped e2, double timer1);

/*
int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_velocity");
	
	ros::NodeHandle td;
	
	ros::Publisher chatter_pub = td.advertise<std_msgs::Float64>("chatter", 1000);
	
	ros::Rate loop_rate(10);
	
	int count = 0;
	std::string name;
	while(ros::ok())
	{
		std_msgs::Float64 msg;
		
		msg.data = count;
		
		chatter_pub.publish(msg);
		
		ROS_INFO("What 's your name ?");
		std::cout << "[  ME ] : I'm ";
		std::getline (std::cin,name);
		//std::cout << "Hello, " << name << "!\n";
		ROS_INFO("Hello, %s!\n", name.c_str());
		
		ros::spinOnce();
		
		loop_rate.sleep();
		++count;
	}
	
	return 0;
}
*/