#include <iostream>
#include <string>

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMavFrame.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/BatteryState.h>

#include <test_velocity/vel_lib.h>
#include <test_velocity/captureGraph.h>

geometry_msgs::PoseStamped target_pose;

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
	ros::Subscriber local_pose_pub = td.subscribe<geometry_msgs::PoseStamped>
			("mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber batt_sub = td.subscribe<sensor_msgs::BatteryState> 
            ("mavros/battery", 10, battery_cb);

	// Publisher
	ros::Publisher local_pos_pub = td.advertise<geometry_msgs::PoseStamped>
			("mavros/setpoint_position/local", 10);
	ros::Publisher vel_sp_pub = td.advertise<geometry_msgs::Twist>
			("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
	
	// Service
	ros::ServiceClient arming_client = td.serviceClient<mavros_msgs::CommandBool>
			("mavros/cmd/arming");
	ros::ServiceClient set_mode_client = td.serviceClient<mavros_msgs::SetMode>
			("mavros/set_mode");
	ros::ServiceClient set_mav_frame_client = td.serviceClient<mavros_msgs::SetMavFrame>
			("mavros/setpoint_velocity/mav_frame");
	
	
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    ROS_INFO("Waiting for FCU connection .");
    while(ros::ok() && current_state.connected){
        //ROS_INFO_ONCE("Waiting for FCU connection ...");
    	std::cout << ".";
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU connected");
    
    // check current pose
    float batt_percent;
    for(int i = 100; ros::ok() && i > 0; --i){
    	ROS_INFO_STREAM("\nCurrent position: \n" << current_pose.pose.position);

    	float batt_percent = current_batt.percentage * 100;
    	ROS_INFO_STREAM("Current Battery: " << batt_percent << "%");
    	
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose_A, pose_B;
    pose_A.pose.position.x = current_pose.pose.position.x;
    pose_A.pose.position.y = current_pose.pose.position.y;
    pose_A.pose.position.z = current_pose.pose.position.z + 3;
    
    geometry_msgs::Twist vs;
    vs.linear.x = 0;
    vs.linear.y = 0;
    vs.linear.z = 0;
    vs.angular.x = 0;
    vs.angular.y = 0;
    vs.angular.z = 0;

    mavros_msgs::SetMode offb_set_mode;
    
    mavros_msgs::SetMavFrame mav_frame_set;
    mav_frame_set.request.mav_frame = 8;
    //set_mav_frame_client.call(mav_frame_set);
    
    ROS_INFO_STREAM("Are you run by velocity controller ? (y/n)");
    char a[100];
	int count = 0;
    std::cin >> a;
    while (1)
    {
    	if (count > 10)
			ros::shutdown();
    		
    	if (strcmp(a, "y")||strcmp(a, "Y"))
    		break;
    	else
    	{
    		ROS_WARN("Can you retype your choice ?");
    	    ROS_INFO_STREAM("Are you run by velocity controller ? (y/n)");
    	}
    	count++;
    }
    
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
    	pose_A.header.stamp = ros::Time::now();
        local_pos_pub.publish(pose_A);
        ros::spinOnce();
        rate.sleep();
    }
    
    ROS_INFO("Ready");
    ros::Duration(3).sleep();
    
    // publish target, keep drone hovering
    int mode = 1;
    //compute velocity to fly from A to B
    vs = compute_velocity(pose_A, pose_B, _timer);

    std::vector<double> v;
    count = 0;
    double origin = ros::Time::now().toSec();
    while(ros::ok())
    {
    	switch(mode)
    	{
    	case 1:
        	ROS_INFO_STREAM("\nCurrent position: \n" << current_pose.pose.position);	
            batt_percent = current_batt.percentage * 100;
            ROS_INFO_STREAM("Current Battery: " << batt_percent << "%");

            local_pos_pub.publish(pose_A);
            
            if( position_distance(current_pose, pose_A) == true)
            {
            	Eigen::Vector3d cur1;
            	tf::pointMsgToEigen(current_pose.pose.position, cur1);
            	ROS_WARN("Use velocity to fly from A to B");
                pose_B.pose.position.x = pose_A.pose.position.x + 4;
                pose_B.pose.position.y = pose_A.pose.position.y;
                pose_B.pose.position.z = pose_A.pose.position.z;
            	mode = 2;
            }
            break;
    	case 2:
    		set_mav_frame_client.call(mav_frame_set);
    		vel_sp_pub.publish(vs);

    		v.push_back(distance(current_pose, pose_B));
    		if( count % 2 == 0)
    			ROS_INFO("Distance between current position and point B: %f (m)", v[count]);

    		//if( count > 1 && v[count] < 0.1 && v[count - 1] < v[count])
    		//{
    		    //ROS_INFO("Travel real time: %6.6f (s)", ros::Time::now().toSec() - origin);
    		//	break;
    		//}
    		if( count > 1 && v[count] < 5 && v[count - 1] < v[count])
    		{
    		    ROS_INFO("Travel real time: %6.6f (s)", ros::Time::now().toSec() - origin);
    			ROS_WARN("The closest distance between the current position and the target position is %f (m)", v[count]);

    			if( v[count] < 0.1)
    				ROS_INFO("Post velocity was successfully");
    			else{
    				ROS_INFO("Post velocity was unsuccessfully");
    			}
    		    mode = 3;
    		    break;
    		}
    		++count;
    		batt_percent = current_batt.percentage * 100;
    		ROS_INFO_STREAM("Current Battery: " << batt_percent << "%");
    		break;
    	case 3:
    	    offb_set_mode.request.custom_mode = "AUTO.LAND";
    	    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
    	    	ROS_INFO("AUTO.LAND enabled");
    	    	mode = 4;
    	    	break;
    	    }
    	}
        if (mode == 4)
        	break;
        ros::spinOnce();
        rate.sleep();
    }

    /*
     * Export image about distance between current position and point B in flight
     */
	std::vector<double> xs1, ys1, xs2, ys2;
    /////////////////////////////////////////////////////-----image 1st-----
    for (int i = 0; i < v.size(); i++)
    {
    	xs1.push_back(i);
    	ys1.push_back(v.at(i));
    }
    std::string name1 = getName();
    captureGraph(xs1, ys1, name1);
	/////////////////////////////////////////////////////-----image 2rd-----
    for (int i = v.size()-10; i < v.size(); i++)
	{
	    xs2.push_back(i);
	    ys2.push_back(v.at(i));
	}
    std::string name2 = getName();
    captureGraph(xs2, ys2, name2);

	return 0;
}
