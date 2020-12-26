#include <iostream>
#include <array>
#include <random>
#include <vector>

#include <ros/ros.h>
#include <angles/angles.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMavFrame.h>
#include <eigen_conversions/eigen_msg.h>

#include <control_velpid/velpid_lib.h>
#include <control_velpid/pid_controller.h>

#include <control_velpid/captureGraph.h>

geometry_msgs::PoseStamped pose_A, pose_B;
geometry_msgs::PoseStamped ps;
geometry_msgs::TwistStamped vs;
Eigen::Vector3d current, dest;

int count = 0, numxx=0;

double distance;
//double err_th = threshold[rand() % threshold.size()];
double err_th = 0.1;
bool stop;

ros::Time last_time;

int main(int argc, char **argv)
{	
	/*
	 * ROS INITIALIZATION
	 */
	ros::init(argc, argv, "test_velocity");
	ros::NodeHandle td;
	
	// Subcriber
    ros::Subscriber state_sub = td.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
	ros::Subscriber local_pose_sub = td.subscribe<geometry_msgs::PoseStamped>
			("/mavros/local_position/pose", 10, local_pos_cb);
	ros::Subscriber batt_sub = td.subscribe<sensor_msgs::BatteryState> 
            ("mavros/battery", 10, battery_cb);

	ros::Publisher local_pos_sp_pub = td.advertise<geometry_msgs::PoseStamped>
			("/mavros/setpoint_position/local", 10);
	ros::Publisher vel_sp_pub = td.advertise<geometry_msgs::TwistStamped>
			("/mavros/setpoint_velocity/cmd_vel", 10);	

	ros::ServiceClient set_mode_client = td.serviceClient<mavros_msgs::SetMode>
			("mavros/set_mode");
	ros::ServiceClient arming_client = td.serviceClient<mavros_msgs::CommandBool>
			("mavros/cmd/arming");
	ros::ServiceClient set_mav_frame_client = td.serviceClient<mavros_msgs::SetMavFrame>
			("mavros/setpoint_velocity/mav_frame");
	
	
	threshold = threshold_definition();
	
	// Linear velocity PID gains and bound of integral windup
	double rate = 20.0;
	double linvel_p_gain = 0.4;
	double linvel_i_gain = 0.05;
	double linvel_d_gain = 0.12;
	double linvel_i_max = 0.1;
	double linvel_i_min = -0.1;

	// Setup of the PID controllers
	setup_livel_pid(linvel_p_gain, linvel_i_gain, linvel_d_gain, linvel_i_max, linvel_i_min);
	
	//the setpoint publishing rate MUST be faster than 2Hz
	ros::Rate loop_rate(rate);

    // wait for FCU connection
    ROS_INFO("Waiting for FCU connection .");
    while(ros::ok() && current_state.connected){
        //ROS_INFO_ONCE("Waiting for FCU connection ...");
    	std::cout << ".";
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("FCU connected");
    
    // check current pose
    float batt_percent;
    for(int i = 100; ros::ok() && i > 0; --i){
    	ROS_INFO_STREAM("\nCurrent position: \n" << current_pose.pose.position);

    	float batt_percent = current_batt.percentage * 100;
    	ROS_INFO_STREAM("Current Battery: " << batt_percent << "%");
    	
        ros::spinOnce();
        loop_rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    
    mavros_msgs::SetMavFrame mav_frame_set;
    mav_frame_set.request.mav_frame = 8;
    //set_mav_frame_client.call(mav_frame_set);
    
    ROS_INFO_STREAM("Are you run by PID - SQUARE ? (y/n)");
    char a[100];
	int count = 0;
    std::cin >> a;
    while (1)
    {
    	if (count > 10)
    		ros::shutdown();
    		
    	if (strcmp(a, "y") == 0||strcmp(a, "Y") == 0)
    		break;
    	else
    	{
    		ROS_WARN("Can you retype your choice ?");
    	    ROS_INFO_STREAM("Are you run by PID - SQUARE ? (y/n)");
    	    std::cin >> a;
    	}
    	count++;
    }
    
    Eigen::Vector3d value_A;
    pose_A.pose.position.x = current_pose.pose.position.x;
    pose_A.pose.position.y = current_pose.pose.position.y;
    pose_A.pose.position.z = current_pose.pose.position.z + 2;
    tf::pointMsgToEigen(pose_A.pose.position, value_A);
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
    	pose_A.header.stamp = ros::Time::now();
    	local_pos_sp_pub.publish(pose_A);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    ROS_INFO("Ready");
    ros::Duration(3).sleep();
    
    // publish target, keep drone hovering
    int mode = 1;

	uint8_t pos_target = 1;

	ROS_INFO("Testing...");
	std::vector<double> data_x, data_y, data_z, data_count;
	std::vector<double> vtoc_x, vtoc_y, vtoc_z, vtoc_c;
	double origin;
	Eigen::Vector3d one;
	while (ros::ok()) {
    	switch(mode)
    	{
    	case 1:
    		ROS_INFO_STREAM("\nCurrent position: \n" << current_pose.pose.position);
    		batt_percent = current_batt.percentage * 100;
    		ROS_INFO_STREAM("Current Battery: " << batt_percent << "%");

    		local_pos_sp_pub.publish(pose_A);
    		loop_rate.sleep();
    		ros::spinOnce();
    		
    		if( position_distance(current_pose, pose_A) == true)
    		{
    		   	ROS_WARN("Use PID velocity to fly from A to B");
    		   	
    		    set_mav_frame_client.call(mav_frame_set);
    		    origin = ros::Time::now().toSec();
    		    mode = 2;
    		}
    		break;
    	case 2:
			// motion routine
			tf::pointMsgToEigen(current_pose.pose.position, one);
			ROS_WARN("Current position: (%f,%f,%f)", one.x(), one.y(), one.z());
		
			switch (pos_target) {
			case 1:
				tf::pointEigenToMsg(pos_setpoint(value_A.x() + 2, value_A.y() + 2, 2), ps.pose.position);
				break;
			case 2:
				tf::pointEigenToMsg(pos_setpoint(value_A.x() - 2, value_A.y() + 2, 2), ps.pose.position);
				break;
			case 3:
				tf::pointEigenToMsg(pos_setpoint(value_A.x() - 2, value_A.y() - 2, 2), ps.pose.position);
				break;
			case 4:
				tf::pointEigenToMsg(pos_setpoint(value_A.x() + 2, value_A.y() - 2, 2), ps.pose.position);
				break;
			case 5:
				tf::pointEigenToMsg(pos_setpoint(value_A.x() + 2, value_A.y() + 2, 2), ps.pose.position);
				break;
			default:
				break;
			}
		
			last_time = ros::Time::now();
			stop = false;

			//ROS_DEBUG("Next setpoint: accepted error threshold: %1.3f", err_th);
			ROS_WARN("Next setpoint: accepted error threshold: %1.3f (m)", err_th);
			//
			// Defines the accepted threshold to the destination/target position before moving to the next setpoint.
			//
			while (!stop) {
				tf::pointMsgToEigen(ps.pose.position, dest);
				tf::pointMsgToEigen(current_pose.pose.position, current);
			
				if (numxx %5 == 0)
				{
					// push data to draw graph
					data_x.push_back(current.x());
					data_y.push_back(current.y());
					data_z.push_back(current.z());
					data_count.push_back(count);
					ROS_INFO_STREAM("\nCurrent position: \n" << current_pose.pose.position);
					batt_percent = current_batt.percentage * 100;
					ROS_INFO_STREAM("Current Battery: " << batt_percent << "%");
				}
				
				numxx++;
			
				distance = sqrt((dest - current).x() * (dest - current).x() + 
								(dest - current).y() * (dest - current).y() + 
								(dest - current).z() * (dest - current).z());

				if (distance <= err_th)
					stop = true;

				tf::vectorEigenToMsg(compute_linvel_effort(dest, current, last_time), vs.twist.linear);
				if ((numxx-1) %5 == 0)
				{
					// push data to draw graph
					vtoc_x.push_back(vs.twist.linear.x);
					vtoc_y.push_back(vs.twist.linear.y);
					vtoc_z.push_back(vs.twist.linear.z);
					vtoc_c.push_back(count++);
				}

				vel_sp_pub.publish(vs);

				last_time = ros::Time::now();
				loop_rate.sleep();
				ros::spinOnce();
			}

			if (pos_target == 6) {
				ROS_INFO("Test complete!");
				ROS_INFO("Travel real time: %6.6f (s)", ros::Time::now().toSec() - origin);
				
				offb_set_mode.request.custom_mode = "AUTO.LAND";
    	    	if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
    	    		ROS_INFO("AUTO.LAND enabled");				

				//Export image path flight
				std::string name1 = getName();
				captureGraph(data_x, data_y, "toado_xy" + name1, 1280, 1280);
				std::string name2 = getName();
				captureGraph(data_count, data_z, "toado_z" + name2, 1280, 360);

				std::string name3 = getName();
				captureGraph(data_x, data_y, "vantoc_xy" + name3, 1280, 1280);
				std::string name4 = getName();
				captureGraph(data_count, data_z, "vantoc_z" + name4, 1280, 360);

				++pos_target;
			}
			else
				++pos_target;
			break;
    	}
		if (pos_target == 7) 
			break;
	}

	return 0;
}
