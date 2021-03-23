#include <iostream>
#include <array>
#include <random>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMavFrame.h>
#include <angles/angles.h>
#include <eigen_conversions/eigen_msg.h>

#include <control_velpid/velpid_lib.h>
#include <control_velpid/pid_controllerv2.h>

#include <control_velpid/captureGraph.h>

Eigen::Vector3d velocity;
double yaw_body;

int count = 0, numxx=0;
bool flight = false;

geometry_msgs::TwistStamped data_vel;
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	data_vel = *msg;
}

int main( int argc, char **argv)
{
	//
	// ROS INITIALIZATION
	//
	ros::init(argc, argv, "control_velocity");
	ros::NodeHandle td;

    ros::Subscriber state_sub = td.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
	ros::ServiceClient arming_client = td.serviceClient<mavros_msgs::CommandBool>
			("mavros/cmd/arming");
	ros::ServiceClient set_mode_client = td.serviceClient<mavros_msgs::SetMode>
			("mavros/set_mode");

	ros::Subscriber local_pose_sub = td.subscribe<geometry_msgs::PoseStamped>
			("/mavros/local_position/pose", 10, local_pos_cb);
	ros::Subscriber vel_sub = td.subscribe<geometry_msgs::TwistStamped>
			("/mavros/local_position/velocity_local", 10, vel_cb);

	ros::Publisher local_pos_sp_pub = td.advertise<geometry_msgs::PoseStamped>
			("/mavros/setpoint_position/local", 10);
	ros::Publisher vel_sp_pub = td.advertise<geometry_msgs::TwistStamped>
			("/mavros/setpoint_velocity/cmd_vel", 10);

	ros::ServiceClient set_mav_frame_client = td.serviceClient<mavros_msgs::SetMavFrame>
			("mavros/setpoint_velocity/mav_frame");

	// threshold = threshold_definition();

	// the setpoint publishing rate MUST be faster than 2Hz
	int rate = 20;
	ros::Rate loop_rate(rate);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        loop_rate.sleep();
    }

    geometry_msgs::PoseStamped pose_A;
    pose_A.pose.position.x = 0;
    pose_A.pose.position.y = 0;
    pose_A.pose.position.z = 3;

    geometry_msgs::PoseStamped pose_B;
    pose_B.pose.position.x = pose_goal(0);
    pose_B.pose.position.y = pose_goal(1);
    pose_B.pose.position.z = pose_goal(2);

    geometry_msgs::TwistStamped vel_msg;
    double _z;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
    	local_pos_sp_pub.publish(pose_A);
        ros::spinOnce();
        loop_rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    //add
    mavros_msgs::SetMavFrame mav_frame_set;
    mav_frame_set.request.mav_frame = 8;

    ros::Time last_request = ros::Time::now();
    ros::Time time_2;

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
                    ROS_INFO("Post position at (0,0,3)");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_sp_pub.publish(pose_A);

        if( position_distance(current_pose, pose_A) == true && ros::Time::now() - time_2 > ros::Duration(5.0))
        {
        	_z = data_vel.twist.linear.z;
        	std::cout << "velocity z : " << _z << std::endl;
        	ROS_INFO("Post velocity");
        	break;
        }
        else if (position_distance(current_pose, pose_A) == false)
        	time_2 = ros::Time::now();

        ros::spinOnce();
        loop_rate.sleep();
    }

    last_request = ros::Time::now();
    while(ros::ok())
    {
    	// velocity(0), velocity(1), velocity(2), yaw = compute_linvel_effort_v2(pose_B, current_pose, last_request);
    	velocity(0), velocity(1), velocity(2) = compute_linvel_effort_v2(pose_B, current_pose, last_request);
        last_request = ros::Time::now();
    	// velocity << 0, 1, 0;
    	if (ros::Time::now() - time_2 > ros::Duration(15))
    		std::cout << current_pose << std::endl;
    	vel_msg.twist.linear.x = velocity(0);
    	vel_msg.twist.linear.y = velocity(1);
    	vel_msg.twist.linear.z = velocity(2);
    	vel_msg.twist.angular.x = 0;
    	vel_msg.twist.angular.y = 0;
    	vel_msg.twist.angular.z = 0;
    	vel_sp_pub.publish(vel_msg);
    	if (position_distance(current_pose, pose_B) == true)
    		break;

        ros::spinOnce();
        loop_rate.sleep();
    }

    offb_set_mode.request.custom_mode = "AUTO.LAND";
	if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
		ROS_INFO("AUTO.LAND enabled");

	return 0;
}
