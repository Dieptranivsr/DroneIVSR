
#include <iostream>
#include <array>
#include <random>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMavFrame.h>
#include <angles/angles.h>
#include <eigen_conversions/eigen_msg.h>
#include <visualization_msgs/Marker.h>

#include <control_velpid/velpid_lib.h>
#include <control_velpid/pid_controller.h>

#include <control_velpid/captureGraph.h>

Eigen::Vector3d velocity;
double yaw_body;

int count = 0, numxx=0;
int target = 1;

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

	ros::Publisher marker_pub = td.advertise<visualization_msgs::Marker>
	    	("visualization_marker", 10);

	//ros::Publisher pub_vel_x = td.advertise<std_msgs::Float64>("/velocity/x",10);
	//ros::Publisher pub_vel_y = td.advertise<std_msgs::Float64>("/velocity/y",10);
	//ros::Publisher pub_vel_z = td.advertise<std_msgs::Float64>("/velocity/z",10);

	//std_msgs::Float64 vel_x;
	//std_msgs::Float64 vel_y;
	//std_msgs::Float64 vel_z;

	// threshold = threshold_definition();

	// the setpoint publishing rate MUST be faster than 2Hz
	int rate = 20;
	ros::Rate loop_rate(rate);

	double linvel_p_gain = 0.4;         //1.4, 0.8
	double linvel_i_gain = 0.05;        //-------- 0,1           0.2
	double linvel_d_gain = 0.12;        //------------ 0.24, 0.2 0.4
	double linvel_i_max = 0.1;
	double linvel_i_min = -0.1;
	setup_livel_pid(linvel_p_gain, linvel_i_gain, linvel_d_gain, linvel_i_max, linvel_i_min);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        loop_rate.sleep();
    }

    //Eigen::Vector4d pose_goal(0, 5.0, 3, -91*M_PI/180);   (0,5,5)
    //Eigen::Vector4d pose_goal(0, -5.0, 12, -91*M_PI/180);

    geometry_msgs::PoseStamped pose_A;
    pose_A.pose.position.x = 0;
    pose_A.pose.position.y = 0;
    pose_A.pose.position.z = 3;

    geometry_msgs::PoseStamped pose_B;
    //pose_B.pose.position.x = pose_goal(0);
    //pose_B.pose.position.y = pose_goal(1);
    //pose_B.pose.position.z = pose_goal(2);

    geometry_msgs::TwistStamped vel_msg;
    double _z;

    visualization_msgs::Marker points, line_strip;
    points.header.frame_id = line_strip.header.frame_id = "map";
    points.header.stamp = line_strip.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = "points_and_lines";
    points.action = line_strip.action = visualization_msgs::Marker::ADD;
    points.id = 0;
    line_strip.id = 1;
    points.type = visualization_msgs::Marker::SPHERE_LIST;         // POINTS, SPHERE_LIST
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.05;         // 0.05, 0.1
    points.scale.y = 0.05;
    points.scale.z = 0.05;

    line_strip.scale.x = 0.03;
    line_strip.scale.x = 0.03;
    line_strip.scale.x = 0.03;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.r = 1.0;
    line_strip.color.a = 1.0;

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

        // if( _position_distance(current_pose, pose_A) < 0.06 && ros::Time::now() - time_2 > ros::Duration(5.0))
        if( _position_distance(current_pose, pose_A) < 0.06 )
        {
        	ROS_INFO("Post velocity");
        	break;
        }
        // else if (position_distance(current_pose, pose_A) == false)
        //	time_2 = ros::Time::now();

        ros::spinOnce();
        loop_rate.sleep();

        points.points.push_back(current_pose.pose.position);
        line_strip.points.push_back(current_pose.pose.position);
        marker_pub.publish(points);
        marker_pub.publish(line_strip);
    }

    ros::Time last_time = ros::Time::now();
    Eigen::Vector3d dest;
    Eigen::Vector3d current;
    while(ros::ok())
    {
    	switch(target)
    	{
    	case 1:
    		pose_B.pose.position.x = 5;
    		pose_B.pose.position.y = 5;
    		pose_B.pose.position.z = 3;
    		break;
    	case 2:
    		pose_B.pose.position.x = -5;
    		pose_B.pose.position.y = 5;
    		pose_B.pose.position.z = 3;
    		break;
    	case 3:
    		pose_B.pose.position.x = -5;
    		pose_B.pose.position.y = -5;
    		pose_B.pose.position.z = 3;
    		break;
    	case 4:
    		pose_B.pose.position.x = 5;
    		pose_B.pose.position.y = -5;
    		pose_B.pose.position.z = 3;
    		break;
    	}

		std::cout << current_pose << std::endl;
		tf::pointMsgToEigen(pose_B.pose.position, dest);
		tf::pointMsgToEigen(current_pose.pose.position, current);
		tf::vectorEigenToMsg(compute_linvel_effort(dest, current, last_time), vel_msg.twist.linear);
    	vel_sp_pub.publish(vel_msg);
		last_time = ros::Time::now();
		//vel_x.data = vel_msg.twist.linear.x;
		//vel_y.data = vel_msg.twist.linear.y;
		//vel_z.data = vel_msg.twist.linear.z;
		//pub_vel_x.publish(vel_x);
		//pub_vel_y.publish(vel_y);
		//pub_vel_z.publish(vel_z);
    	if (_position_distance(pose_B, current_pose) < 0.1 )
    	{
    		if (target == 4)
    		{
    		    offb_set_mode.request.custom_mode = "AUTO.LAND";
    			if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
    				ROS_INFO("AUTO.LAND enabled");
    			break;
    		}
    		else
    			target++;
    	}

        ros::spinOnce();
        loop_rate.sleep();

        points.points.push_back(current_pose.pose.position);
        line_strip.points.push_back(current_pose.pose.position);
        marker_pub.publish(points);
        marker_pub.publish(line_strip);
    }

	return 0;
}

/*
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
#include <visualization_msgs/Marker.h>

#include <control_velpid/velpid_lib.h>
#include <control_velpid/pid_controller.h>

#include <control_velpid/captureGraph.h>

geometry_msgs::PoseStamped ps;
geometry_msgs::TwistStamped vs;
Eigen::Vector3d current;

int count = 0, numxx=0;

int main(int argc, char **argv)
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
	ros::Publisher local_pos_sp_pub = td.advertise<geometry_msgs::PoseStamped>
			("/mavros/setpoint_position/local", 10);
	ros::Publisher vel_sp_pub = td.advertise<geometry_msgs::TwistStamped>
			("/mavros/setpoint_velocity/cmd_vel", 10);

	ros::Publisher marker_pub = td.advertise<visualization_msgs::Marker>
	    		("visualization_marker", 10);

	threshold = threshold_definition();

	double rate = 20.0;
	double linvel_p_gain = 0.4;         //1.4, 0.8
	double linvel_i_gain = 0.05;        //-------- 0,1           0.2
	double linvel_d_gain = 0.12;        //------------ 0.24, 0.2 0.4
	double linvel_i_max = 0.1;
	double linvel_i_min = -0.1;

	// Setup of the PID controllers
	setup_livel_pid(linvel_p_gain, linvel_i_gain, linvel_d_gain, linvel_i_max, linvel_i_min);

	//the setpoint publishing rate MUST be faster than 2Hz
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

    visualization_msgs::Marker points, line_strip;
    points.header.frame_id = line_strip.header.frame_id = "map";
    points.header.stamp = line_strip.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = "points_and_lines";
    points.action = line_strip.action = visualization_msgs::Marker::ADD;
    points.id = 0;
    line_strip.id = 1;
    points.type = visualization_msgs::Marker::SPHERE_LIST;         // POINTS, SPHERE_LIST
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.05;         // 0.05, 0.1
    points.scale.y = 0.05;
    points.scale.z = 0.05;

    line_strip.scale.x = 0.03;
    line_strip.scale.x = 0.03;
    line_strip.scale.x = 0.03;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.r = 1.0;
    line_strip.color.a = 1.0;

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

    ros::Time last_request = ros::Time::now();
    double origin;  //Compute time of flight

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

        if( position_distance(current_pose, pose_A) == true)
        {
        	Eigen::Vector3d cur1;
        	tf::pointMsgToEigen(current_pose.pose.position, cur1);
        	ROS_INFO("Current position at point A : (%f,%f,%f)", cur1.x(), cur1.y(), cur1.z());
        	ROS_INFO("Use velocity to fly from A");
        	origin = ros::Time::now().toSec();
        	break;
        }

        ros::spinOnce();
        loop_rate.sleep();

        points.points.push_back(current_pose.pose.position);
        line_strip.points.push_back(current_pose.pose.position);
        marker_pub.publish(points);
        marker_pub.publish(line_strip);
    }

	uint8_t pos_target = 1;

	ROS_INFO("Testing...");
	std::vector<double> xs1, ys1, zs1, ts1;
	std::vector<double> vel_x, vel_y, vel_z;
	while (ros::ok()) {
		//set_mav_frame_client.call(mav_frame_set);
		// motion routine
        Eigen::Vector3d one;
        tf::pointMsgToEigen(current_pose.pose.position, one);
        ROS_INFO("Current position: (%f,%f,%f)", one.x(), one.y(), one.z());

		switch (pos_target) {
		case 1:
			tf::pointEigenToMsg(pos_setpoint(3, 3, 3), ps.pose.position);
			break;
		case 2:
			tf::pointEigenToMsg(pos_setpoint(-3, 3, 3), ps.pose.position);
			break;
		case 3:
			tf::pointEigenToMsg(pos_setpoint(-3, -3, 3), ps.pose.position);
			break;
		case 4:
			tf::pointEigenToMsg(pos_setpoint(3, -3, 3), ps.pose.position);
			break;
		case 5:
			tf::pointEigenToMsg(pos_setpoint(3, 3, 3), ps.pose.position);
			break;
		default:
			break;
		}

		ros::Time last_time = ros::Time::now();
		bool stop;
		stop = false;

		Eigen::Vector3d dest;

		double distance;
		//double err_th = threshold[rand() % threshold.size()];
		double err_th = 0.1;

		ROS_DEBUG("Next setpoint: accepted error threshold: %1.3f", err_th);
		ROS_WARN("Next setpoint: accepted error threshold: %1.3f (m)", err_th);
		//
		// Defines the accepted threshold to the destination/target position before moving to the next setpoint.
		//
		while (ros::ok() && !stop) {
			tf::pointMsgToEigen(ps.pose.position, dest);
			tf::pointMsgToEigen(current_pose.pose.position, current);

			if (numxx %10 == 0)
			{
				// push data to draw graph
				xs1.push_back(current.x());
				ys1.push_back(current.y());
				zs1.push_back(current.z());
				ts1.push_back(count++);

				vel_x.push_back(vs.twist.linear.x);
				vel_y.push_back(vs.twist.linear.y);
				vel_z.push_back(vs.twist.linear.z);
				ROS_INFO("Current position: (%f,%f,%f)", current.x(), current.y(), current.z());
			}

			numxx++;

			distance = sqrt((dest - current).x() * (dest - current).x() +
							(dest - current).y() * (dest - current).y() +
							(dest - current).z() * (dest - current).z());

			if (distance <= err_th)
				stop = true;

			tf::vectorEigenToMsg(compute_linvel_effort(dest, current, last_time), vs.twist.linear);

			vel_sp_pub.publish(vs);

			last_time = ros::Time::now();
			loop_rate.sleep();
			ros::spinOnce();

	        points.points.push_back(current_pose.pose.position);
	        line_strip.points.push_back(current_pose.pose.position);
	        marker_pub.publish(points);
	        marker_pub.publish(line_strip);
		}

		if (pos_target == 6) {
			ROS_INFO("Test complete!");
			ROS_INFO("Travel real time: %6.6f (s)", ros::Time::now().toSec() - origin);

			//Export image path flight
			std::string name1 = getName();
			captureGraph(xs1, ys1, name1 + "toado_xy", 1280, 1280);
			captureGraph(zs1, ts1, name1 + "toado_z", 1280, 360);

			captureGraph(ts1, vel_x, name1 + "vantoc_x", 1280, 360);
			captureGraph(ts1, vel_y, name1 + "vantoc_y", 1280, 360);
			captureGraph(ts1, vel_z, name1 + "vantoc_z", 1280, 360);

		    // Shutdown Drone
			pos_target = 7;
			offb_set_mode.request.custom_mode = "AUTO.LAND";
			if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
				ROS_INFO("AUTO.LAND enabled");
		}
		else
			++pos_target;

		loop_rate.sleep();
		ros::spinOnce();

        points.points.push_back(current_pose.pose.position);
        line_strip.points.push_back(current_pose.pose.position);
        marker_pub.publish(points);
        marker_pub.publish(line_strip);
	}

	return 0;
}
*/
