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
#include <control_velpid/pid_controller.h>

#include <control_velpid/captureGraph.h>

geometry_msgs::PoseStamped ps;
geometry_msgs::TwistStamped vs;
Eigen::Vector3d current;

int count = 0, numxx=0;

/*
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
	
	ros::ServiceClient set_mav_frame_client = td.serviceClient<mavros_msgs::SetMavFrame>
			("mavros/setpoint_velocity/mav_frame");
	
	threshold = threshold_definition();
	
	double rate = 20.0;
	double linvel_p_gain = 0.4;
	double linvel_i_gain = 0.05;
	double linvel_d_gain = 0.12;
	double linvel_i_max = 0.1;
	double linvel_i_min = -0.1;

	// Linear velocity PID gains and bound of integral windup	
	//double linvel_p_gain;
	//double linvel_i_gain;
	//double linvel_d_gain;
	//double linvel_i_max;
	//double linvel_i_min;
	//td.param("linvel_p_gain", linvel_p_gain, 0.4);
	//td.param("linvel_i_gain", linvel_i_gain, 0.05);
	//td.param("linvel_d_gain", linvel_d_gain, 0.12);
	//td.param("linvel_i_max", linvel_i_max, 0.1);
	//td.param("linvel_i_min", linvel_i_min, -0.1);	

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
    pose_A.pose.position.z = 2;
    
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
                    ROS_INFO("Post position at (0,0,2)");
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
    }
    
	uint8_t pos_target = 1;

	ROS_INFO("Testing...");
	std::vector<double> xs1, ys1, zs1, ts1;
	while (ros::ok()) {
		set_mav_frame_client.call(mav_frame_set);
		// motion routine
        Eigen::Vector3d one;
        tf::pointMsgToEigen(current_pose.pose.position, one);
        ROS_INFO("Current position: (%f,%f,%f)", one.x(), one.y(), one.z());
		
		switch (pos_target) {
		case 1:
			tf::pointEigenToMsg(pos_setpoint(1, 1, 1), ps.pose.position);
			break;
		case 2:
			tf::pointEigenToMsg(pos_setpoint(-1, 1, 1), ps.pose.position);
			break;
		case 3:
			tf::pointEigenToMsg(pos_setpoint(-1, -1, 1), ps.pose.position);
			break;
		case 4:
			tf::pointEigenToMsg(pos_setpoint(1, -1, 1), ps.pose.position);
			break;
		case 5:
			tf::pointEigenToMsg(pos_setpoint(1, 1, 1), ps.pose.position);
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
		}

		if (pos_target == 6) {
			ROS_INFO("Test complete!");
			ROS_INFO("Travel real time: %6.6f (s)", ros::Time::now().toSec() - origin);
			
			//Export image path flight
		    std::string name1 = getName();
		    captureGraph(xs1, ys1, name1);
		    std::string name2 = getName();
		    captureGraph(zs1, ts1, name2);
		    
		    // Shutdown Drone
			ros::shutdown();
		}
		else
			++pos_target;

		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
*/

/*
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
	
	ros::ServiceClient set_mav_frame_client = td.serviceClient<mavros_msgs::SetMavFrame>
			("mavros/setpoint_velocity/mav_frame");
	
	threshold = threshold_definition();
	
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
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        loop_rate.sleep();
    }

    geometry_msgs::PoseStamped pose_A;
    pose_A.pose.position.x = 5;
    pose_A.pose.position.y = 0;
    pose_A.pose.position.z = 1;
    
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
                    ROS_INFO("Post position at (5,0,1)");
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
    }
    
	uint8_t pos_target = 1;

	ROS_INFO("Testing...");
	ros::Time last_time = ros::Time::now();
	std::vector<double> xs1, ys1, zs1, ts1;
	while (ros::ok()) {
		tf::pointMsgToEigen(current_pose.pose.position, current);
		tf::pointEigenToMsg(Eigen::Vector3d(5.0f, 0.0f, 1.0f), ps.pose.position);
		tf::vectorEigenToMsg(compute_linvel_effort(	Eigen::Vector3d(5.0f, 0.0f, 1.0f), current, last_time), vs.twist.linear);
		vel_sp_pub.publish(vs);
		
		bool stop = false;
		
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
			ROS_INFO("Current position: (%f,%f,%f)", current.x(), current.y(), current.z());
			
			distance = sqrt((dest - current).x() * (dest - current).x() + 
							(dest - current).y() * (dest - current).y() + 
							(dest - current).z() * (dest - current).z());

			if (distance <= err_th)
				stop = true;

			//tf::vectorEigenToMsg(compute_linvel_effort(dest, current, last_time), vs.twist.linear);

			//vel_sp_pub.publish(vs);

			last_time = ros::Time::now();
			loop_rate.sleep();
			ros::spinOnce();
		}
		
		// motion routine
		for (int theta = 0; theta <= 720; theta++) {
			tf::pointMsgToEigen(current_pose.pose.position, current);
			
			if (numxx %2 == 0)
			{
				// push data to draw graph
				xs1.push_back(current.x());
				ys1.push_back(current.y());
				zs1.push_back(current.z());
				ts1.push_back(count++);
				ROS_INFO("Current position: (%f,%f,%f)", current.x(), current.y(), current.z());
			}
				
			numxx++;
			
			tf::vectorEigenToMsg(compute_linvel_effort(circle_shape(theta), current, last_time), vs.twist.linear);
			vel_sp_pub.publish(vs);
			if (theta == 520) {
				ROS_INFO("Test complete!");
				ROS_INFO("Travel real time: %6.6f (s)", ros::Time::now().toSec() - origin);
				
				//Export image path flight
			    std::string name1 = getName();
			    captureGraph(xs1, ys1, name1);
			    std::string name2 = getName();
			    captureGraph(zs1, ts1, name2);
			    
			    // Shutdown Drone
				ros::shutdown();
			}
			last_time = ros::Time::now();
			loop_rate.sleep();
			ros::spinOnce();
		}
	}

	return 0;
}
*/


// Square path motion routine
/*
void square_path_motion(ros::Rate loop_rate){
	uint8_t pos_target = 1;

	ROS_INFO("Testing...");

	while (ros::ok()) {
		wait_and_move(ps);

		// motion routine
		switch (pos_target) {
		case 1:
			tf::pointEigenToMsg(pos_setpoint(1, 1, 1), ps.pose.position);
			break;
		case 2:
			tf::pointEigenToMsg(pos_setpoint(-1, 1, 1), ps.pose.position);
			break;
		case 3:
			tf::pointEigenToMsg(pos_setpoint(-1, -1, 1), ps.pose.position);
			break;
		case 4:
			tf::pointEigenToMsg(pos_setpoint(1, -1, 1), ps.pose.position);
			break;
		case 5:
			tf::pointEigenToMsg(pos_setpoint(1, 1, 1), ps.pose.position);
			break;
		default:
			break;
		}

		if (pos_target == 6) {
			ROS_INFO("Test complete!");
			ros::shutdown();
		}
		else
			++pos_target;

		loop_rate.sleep();
		ros::spinOnce();
	}
}
*/


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
	
	ros::ServiceClient set_mav_frame_client = td.serviceClient<mavros_msgs::SetMavFrame>
			("mavros/setpoint_velocity/mav_frame");
	
	threshold = threshold_definition();
	
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
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        loop_rate.sleep();
    }

    geometry_msgs::PoseStamped pose_A;
    pose_A.pose.position.x = 0;
    pose_A.pose.position.y = 0;
    pose_A.pose.position.z = 2;
    
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
                    ROS_INFO("Post position at (0,0,2)");
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
    }
    
	uint8_t pos_target = 1;

	ROS_INFO("Testing...");
	ros::Time last_time = ros::Time::now();
	std::vector<double> xs1, ys1, zs1, ts1;
	while (ros::ok()) {
		tf::pointMsgToEigen(current_pose.pose.position, current);
		tf::pointEigenToMsg(Eigen::Vector3d(0.0f, 0.0f, 2.0f), ps.pose.position);
		tf::vectorEigenToMsg(compute_linvel_effort(	Eigen::Vector3d(0.0f, 0.0f, 2.0f), current, last_time), vs.twist.linear);
		//vel_sp_pub.publish(vs);
		
		bool stop = false;
		
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
			ROS_INFO("Current position: (%f,%f,%f)", current.x(), current.y(), current.z());
			
			distance = sqrt((dest - current).x() * (dest - current).x() + 
							(dest - current).y() * (dest - current).y() + 
							(dest - current).z() * (dest - current).z());

			if (distance <= err_th)
				stop = true;

			//tf::vectorEigenToMsg(compute_linvel_effort(dest, current, last_time), vs.twist.linear);

			//vel_sp_pub.publish(vs);

			last_time = ros::Time::now();
			loop_rate.sleep();
			ros::spinOnce();
		}
		
		// motion routine
		for (int theta = -180; theta <= 360; theta++) {
			tf::pointMsgToEigen(current_pose.pose.position, current);
			
			if (numxx %2 == 0)
			{
				// push data to draw graph
				xs1.push_back(current.x());
				ys1.push_back(current.y());
				zs1.push_back(current.z());
				ts1.push_back(count++);
				ROS_INFO("Current position: (%f,%f,%f)", current.x(), current.y(), current.z());
			}
				
			numxx++;
			
			tf::vectorEigenToMsg(compute_linvel_effort(eight_shape(theta), current, last_time), vs.twist.linear);
			vel_sp_pub.publish(vs);
			if (theta == 340) {
				ROS_INFO("Test complete!");
				ROS_INFO("Travel real time: %6.6f (s)", ros::Time::now().toSec() - origin);
				
			offb_set_mode.request.custom_mode = "AUTO.LAND";
    	    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
    	    	ROS_INFO("AUTO.LAND enabled");								

				//Export image path flight
			    std::string name1 = getName();
			    captureGraph(xs1, ys1, name1, 1280, 1280);
			    std::string name2 = getName();
			    captureGraph(zs1, ts1, name2, 1280, 1280);

			}
			last_time = ros::Time::now();
			loop_rate.sleep();
			ros::spinOnce();
		}
	}

	return 0;
}


/*
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
	
	ros::ServiceClient set_mav_frame_client = td.serviceClient<mavros_msgs::SetMavFrame>
			("mavros/setpoint_velocity/mav_frame");
	
	threshold = threshold_definition();
	
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
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        loop_rate.sleep();
    }

    geometry_msgs::PoseStamped pose_A;
    pose_A.pose.position.x = 0;
    pose_A.pose.position.y = 0;
    pose_A.pose.position.z = 2.5;
    
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
                    ROS_INFO("Post position at (0,0,2.5)");
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
    }
    
	uint8_t pos_target = 1;

	ROS_INFO("Testing...");
	ros::Time last_time = ros::Time::now();
	std::vector<double> xs1, ys1, zs1, ts1;
	while (ros::ok()) {
		tf::pointMsgToEigen(current_pose.pose.position, current);
		tf::pointEigenToMsg(Eigen::Vector3d(0.0f, 0.0f, 2.5f), ps.pose.position);
		tf::vectorEigenToMsg(compute_linvel_effort(	Eigen::Vector3d(0.0f, 0.0f, 2.5f), current, last_time), vs.twist.linear);
		//vel_sp_pub.publish(vs);
		
		bool stop = false;
		
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
			ROS_INFO("Current position: (%f,%f,%f)", current.x(), current.y(), current.z());
			
			distance = sqrt((dest - current).x() * (dest - current).x() + 
							(dest - current).y() * (dest - current).y() + 
							(dest - current).z() * (dest - current).z());

			if (distance <= err_th)
				stop = true;

			//tf::vectorEigenToMsg(compute_linvel_effort(dest, current, last_time), vs.twist.linear);

			//vel_sp_pub.publish(vs);

			last_time = ros::Time::now();
			loop_rate.sleep();
			ros::spinOnce();
		}
		
		// motion routine
		for (int theta = 0; theta <= 720; theta++) {
			tf::pointMsgToEigen(current_pose.pose.position, current);
			
			if (numxx %2 == 0)
			{
				// push data to draw graph
				xs1.push_back(current.x());
				ys1.push_back(current.y());
				zs1.push_back(current.z());
				ts1.push_back(count++);
				ROS_INFO("Current position: (%f,%f,%f)", current.x(), current.y(), current.z());
			}
				
			numxx++;
			
			tf::vectorEigenToMsg(compute_linvel_effort(ellipse_shape(theta), current, last_time), vs.twist.linear);
			vel_sp_pub.publish(vs);
			if (theta == 480) {
				ROS_INFO("Test complete!");
				ROS_INFO("Travel real time: %6.6f (s)", ros::Time::now().toSec() - origin);
				
				//Export image path flight
			    std::string name1 = getName();
			    captureGraph(xs1, ys1, name1);
			    std::string name2 = getName();
			    captureGraph(zs1, ts1, name2);
			    
			    // Shutdown Drone
				ros::shutdown();
			}
			last_time = ros::Time::now();
			loop_rate.sleep();
			ros::spinOnce();
		}
	}

	return 0;
}
*/

// Circle path motion routine
/*
void circle_path_motion(ros::Rate loop_rate){
	ROS_INFO("Testing...");
	ros::Time last_time = ros::Time::now();

	while (ros::ok()) {
		tf::pointMsgToEigen(current_pose.pose.position, current);


		//if (use_pid)
			tf::vectorEigenToMsg(pid.compute_linvel_effort(	Eigen::Vector3d(5.0f, 0.0f, 1.0f), current, last_time), vs.twist.linear);
		//else
		//	tf::vectorEigenToMsg(Eigen::Vector3d(5.0f - current.x(), -current.y(), 1.0f - current.z()), vs.twist.linear);
		vel_sp_pub.publish(vs);

		wait_and_move(ps);

		// motion routine
		for (int theta = 0; theta <= 360; theta++) {
			tf::pointMsgToEigen(current_pose.pose.position, current);

			//if (use_pid)
				tf::vectorEigenToMsg(pid.compute_linvel_effort(circle_shape(theta), current, last_time), vs.twist.linear);
			//else
			//	tf::vectorEigenToMsg(circle_shape(theta) - current, vs.twist.linear);
			vel_sp_pub.publish(vs);

			if (theta == 360) {
				ROS_INFO("Test complete!");
				ros::shutdown();
			}
			last_time = ros::Time::now();
			loop_rate.sleep();
			ros::spinOnce();
		}
	}
}
*/

// Eight path motion routine
/*
void eight_path_motion(ros::Rate loop_rate){
	ROS_INFO("Testing...");
	ros::Time last_time = ros::Time::now();

	while (ros::ok()) {
		tf::pointMsgToEigen(current_pose.pose.position, current);

		// starting point
		//if (use_pid)
			tf::vectorEigenToMsg(pid.compute_linvel_effort( Eigen::Vector3d(0.0f, 0.0f, 1.0f), current, last_time), vs.twist.linear);
		//else
		//	tf::vectorEigenToMsg(Eigen::Vector3d(-current.x(), -current.y(), 1.0f - current.z()), vs.twist.linear);
		vel_sp_pub.publish(vs);

		wait_and_move(ps);

		// motion routine
		for (int theta = -180; theta <= 180; theta++) {
			tf::pointMsgToEigen(current_pose.pose.position, current);

			//if (use_pid)
				tf::vectorEigenToMsg(pid.compute_linvel_effort(eight_shape(theta), current, last_time), vs.twist.linear);
			//else
			//	tf::vectorEigenToMsg(eight_shape(theta) - current, vs.twist.linear);
			vel_sp_pub.publish(vs);
		
			if (theta == 180) {
				ROS_INFO("Test complete!");
				ros::shutdown();
			}
			last_time = ros::Time::now();
			loop_rate.sleep();
			ros::spinOnce();
		}
	}
}
*/

