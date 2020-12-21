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

#include <test_velocity/vel_lib.h>
#include <test_velocity/captureGraph.h>

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
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    
    geometry_msgs::PoseStamped pose_A, pose_B;
    pose_A.pose.position.x = 0;
    pose_A.pose.position.y = 0;
    pose_A.pose.position.z = 2;

    pose_B.pose.position.x = 3;
    pose_B.pose.position.y = 0;
    pose_B.pose.position.z = 2;
    
    geometry_msgs::Twist vs;
    vs.linear.x = 0;
    vs.linear.y = 0;
    vs.linear.z = 0;
    vs.angular.x = 0;
    vs.angular.y = 0;
    vs.angular.z = 0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose_A);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    //add
    mavros_msgs::SetMavFrame mav_frame_set;
    mav_frame_set.request.mav_frame = 8;
    //set_mav_frame_client.call(mav_frame_set);
    
    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                    ROS_INFO("Post position at (0,0,2)");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose_A);

        if( position_distance(current_pose, pose_A) == true)
        {
        	Eigen::Vector3d cur1;
        	tf::pointMsgToEigen(current_pose.pose.position, cur1);
        	ROS_INFO("Current position at point A : (%f,%f,%f)", cur1.x(), cur1.y(), cur1.z());
        	ROS_INFO("Use velocity to fly from A to B");
        	break;
        }
        
        ros::spinOnce();
        rate.sleep();
    }

    //compute velocity to fly from A to B
	vs = compute_velocity(pose_A, pose_B, _timer);

	std::vector<double> v;
	int count = 0;
    last_request = ros::Time::now();
    double origin = ros::Time::now().toSec();
    //while(ros::ok() && ros::Time::now() - last_request <= ros::Duration(timer1 + 0.01)){
    while(ros::ok()){
    	set_mav_frame_client.call(mav_frame_set);
    	vel_sp_pub.publish(vs);
    	
    	ros::spinOnce();
    	rate.sleep();
    	
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
    		break;
    	}
    	++count;
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
	
	Eigen::Vector3d cur2;
	tf::pointMsgToEigen(current_pose.pose.position, cur2);
	ROS_INFO("Current position at point B : (%f,%f,%f)", cur2.x(), cur2.y(), cur2.z());
	ROS_WARN("The closest distance between the current position and the target position is %f (m)", distance(current_pose, pose_B));

	if( position_distance(current_pose, pose_B))
		ROS_INFO("Post velocity was successfully");
	else{
		ROS_INFO("Post velocity was unsuccessfully");
	}
	ros::Duration(5).sleep();
	
    offb_set_mode.request.custom_mode = "AUTO.LAND";
    while(ros::ok()){
    	if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
    		ROS_INFO("AUTO.LAND enabled");
            break;
    	}
    	
    	ros::spinOnce();
        rate.sleep();
    }
    
	return 0;
}


/*
#include <iostream>

#include <ros/ros.h>
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
*/