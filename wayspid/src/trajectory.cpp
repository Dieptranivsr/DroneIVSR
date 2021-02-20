#include "wayspid/trajectory.h"

geometry_msgs::PoseStamped local_attitude;
void dynamicReconfigureCallback(wayspid::setTrajectoryConfig &config, uint32_t level)
{
	std::cout << "[Trajectory] setTrajectoryConfig is running..." << std::endl;
	last_yaw = config.yaw_d / 180 * M_PI;
	trajectory_type = config.trajectory;
	if (level == 0){
		waypoint = 0;
		if (trajectory_type == 6)
		{
			t = M_PI / 2;
		}
		else if (trajectory_type == 7)
		{
			t = 1;
		}
		else 
			t = 0;
	}
	
	speed = config.speed;
	straight_speed = config.straight_speed;
	
	pose_d << config.x_d, config.y_d, config.z_d, config.yaw_d / 180 * M_PI;
	yaw_d = initial_local_yaw + pose_d(3);
}

void attitudeCallback(const geometry_msgs::PoseStamped::ConstPtr& attitude_msg)
{
	local_attitude = *attitude_msg;
}

void commandCallback(const std_msgs::Int8& command_msg)
{
	if (command_msg.data == 5){
		trajectory_type = 11;
	}
}

geometry_msgs::PoseStamped local_pos;
void localPosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	local_pos = *msg;
	pose_act << local_pos.pose.position.x, local_pos.pose.position.y, local_pos.pose.position.z, 0;
}

/*
 * Constructor
 */
Trajectory::Trajectory(int argc, char** argv)
{
	ros::init(argc, argv, "Trajectory");
	ros::NodeHandle td;
	
	// Subscriber
	attitude_sub = td.subscribe<geometry_msgs::PoseStamped>
				("/attitude_RPY/local", 1, attitudeCallback);
	command_sub = td.subscribe("/d9/command", 1, commandCallback);
	local_pos_sub = td.subscribe<geometry_msgs::PoseStamped>
				("mavros/local_position/pose", 10, localPosCallback);
	
	// Pushlisher
	trajectory_pub = td.advertise<geometry_msgs::QuaternionStamped>
				("/uav/trajectory", 1);
	velocity_pub = td.advertise<geometry_msgs::QuaternionStamped>
				("/uav/trajectory_velocity", 1);
	trajectory_type_pub = td.advertise<std_msgs::Int32>
				("/trajectory_type", 1);
	
	// Initializing parameters
	pose_d << 0, 0, 0, 0;
	trajectory_type = 0;
	speed = 1;
	straight_speed = 1;
	
	yaw_d = 0;
	
	// Input txt line
	std::string line;
	std::ifstream myfile(ros::package::getPath("wayspid") + "/policy/waypoints.txt");
	if(myfile.is_open())
	{
		int points;
		while (getline(myfile, line))
			++points;
		waypoints = Eigen::MatrixXd(points, 3);
		std::ifstream myfile(ros::package::getPath("wayspid") + "/policy/waypoints.txt");
		for (int i = 0; getline(myfile, line); ++i)
		{
			std::string delimiter = "\t";
			
			size_t pos = 0;
			for (int j = 0; (pos = line.find(delimiter)) != std::string::npos; ++j)
			{
				waypoints(i, j) = atof(line.substr(0, pos).c_str());
				line.erase(0, pos + delimiter.length());
			}
			waypoints(i, 2) = atof(line.c_str());
		}
		myfile.close();
	}
	else
		std::cout << "Unable to open file: "<< ros::package::getPath("wayspid") + "/policy/waypoints.txt";
	
	waypoint = 0;
}

/*
 * Destructor
 */
Trajectory::~Trajectory()
{
	ros::shutdown();
	exit(0);
}

/*
 * Function to normalize angles from -pi to pi
 */
double Trajectory::denormalizeAngle(double deA)
{
	if (deA > M_PI)
		deA -= 2 * M_PI;
	else if (deA < -M_PI)
		deA += 2 * M_PI;
	else
		return deA;
}


void Trajectory::run()
{
	ros::Rate rate(100);
	
	/*
	 * Take initial yaw angle
	 */
	for (int i = 1000; ros::ok() && i > 0; --i)
	{
		initial_local_yaw = local_attitude.pose.orientation.z * (M_PI / 180);
		ros::spinOnce();
		rate.sleep();
	}
	
	/*
	 * Set dynamic reconfigure (GUI) connections
	 */
    dynamic_reconfigure::Server<wayspid::setTrajectoryConfig> server;
    dynamic_reconfigure::Server<wayspid::setTrajectoryConfig>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);
    
    Eigen::Vector4d trajectory;
    Eigen::Vector4d velocity;
    t = 0;
    t_straight = 0;
    double R = sqrt(2) / 2;
    Eigen::Vector3d w1;
    Eigen::Vector3d w2;
    
    double var_speed = 0;
    double d = 0;
    double x, y;
    
    double dt = (double)1/100;
    
    // main loop
    while (ros::ok())
    {
    	rate.sleep();
    	ros::spinOnce();
    	
    	t += speed * dt;
    	
    	// Normalized yaw
    	yaw_d = denormalizeAngle(yaw_d);
    	
    	Eigen::Matrix3d R1;
    	Eigen::Matrix3d R2;
    	
    	// main switch case
    	switch(trajectory_type)
    	{
    	case 0: // default (no command)
    		trajectory << 0, 0, -100, yaw_d;
    		velocity << 0, 0, 0, 0;
    		break;
    	case 1: // hover
    		trajectory << 0, 0, 2, yaw_d;
    		velocity << 0, 0, 0, 0;
    		break;
    	case 2: // user
    		trajectory << pose_d(0), pose_d(1), pose_d(2), yaw_d;
    		velocity << 0, 0, 0, 0;
    	case 3: // waypoints
    		if (waypoint << waypoints.rows() - 1)
    		{
    			w1 << waypoints.row(waypoint).transpose();
    			w2 << waypoints.row(waypoint + 1).transpose();
    			double d = distance(w1, w2);
    			trajectory << (1 - t / d) * w1 + t / d * w2, yaw_d;
    			x = trajectory(0) * cos (-yaw_d) + trajectory(1) * sin(-yaw_d);
    			x = trajectory(1) * cos (-yaw_d) - trajectory(0) * sin(-yaw_d);
    			trajectory << x, y, trajectory(2), yaw_d;
    			if (t >= d)
    			{
    				t = 0;
    				++waypoint;
    			}
    		}
    		else 
    			trajectory << waypoints.bottomRows(1).transpose(), yaw_d;
    		velocity << 0, 0, 0, 0;
    		break;
        case 4: // agile waypoints --  zig-zag + straight-line [original/go-away]
            if(waypoint < waypoints.rows() - 2){
                //cout << "[Trajectory]: waypoint = " << waypoint << endl;
                w1 << waypoints.row(waypoint).transpose();
                w2 << waypoints.row(waypoint + 1).transpose();
                double d = distance(w1, w2);
                trajectory << (1 - t / d) * w1 + t / d * w2, yaw_d;
                x = trajectory(0)*cos(-yaw_d) + trajectory(1)*sin(-yaw_d);
                y = trajectory(1)*cos(-yaw_d) - trajectory(0)*sin(-yaw_d);
                trajectory << x, y, trajectory(2), yaw_d;
                if(t >= d){
                    t = 0;
                    t_straight = 0;
                    waypoint++;
                }
            }
            else if(waypoint < waypoints.rows() - 1){
            	t_straight += straight_speed * dt;
            	//cout << "[Trajectory]: waypoint = " << waypoint << endl;
            	w1 << waypoints.row(waypoint).transpose();
            	w2 << waypoints.row(waypoint + 1).transpose();
            	double d = distance(w1, w2);
            	trajectory << (1 - t_straight / d) * w1 + t_straight / d * w2, yaw_d;
            	x = trajectory(0)*cos(-yaw_d) + trajectory(1)*sin(-yaw_d);
            	y = trajectory(1)*cos(-yaw_d) - trajectory(0)*sin(-yaw_d);
            	trajectory << x, y, trajectory(2), yaw_d;
            	if(t_straight >= d){
            		t = 0;
            		t_straight = 0;
            		waypoint++;
            	}
            }
            else
                trajectory << waypoints.bottomRows(1).transpose(), yaw_d;
            velocity << 0, 0, 0, 0;
            break;
        case 5: // Circle
            // Uncomment to change yaw while performing circle
            //yaw_d = fmod(t, 2 * M_PI) - M_PI;

            // Slant circle - Uncomment to change z in trajectory
            //trajectory << 3*cos(t), 3*sin(t), pose_d(2) + sin(t), yaw_d;
            //velocity << -speed * sin(t), speed * cos(t), speed * cos(t), 0;

            // Level circle
            trajectory << 3*cos(t), 3*sin(t), pose_d(2), yaw_d;
            velocity << -speed * sin(t), speed * cos(t), 0, 0;
            break;
        case 13: // Vertical circle
            // Uncomment to change yaw while performing circle
            //yaw_d = fmod(t, 2 * M_PI) - M_PI;

            trajectory << trajectory(0), 2*cos(t), pose_d(2) + 2*sin(t), yaw_d;
            velocity << 0, -2*speed * sin(t), 2*speed * cos(t), 0;
            break;
        case 6: // motor failure waypoints -- straight line + mapping
        	if(waypoint < 1){
        		t_straight += straight_speed * dt;
        		//cout << "[Trajectory]: waypoint = " << waypoint << endl;
        		w1 << waypoints.row(waypoint).transpose();
        		w2 << waypoints.row(waypoint + 1).transpose();
        		double d = distance(w1, w2);
        		trajectory << (1 - t_straight / d) * w1 + t_straight / d * w2, yaw_d;
        		x = trajectory(0)*cos(-yaw_d) + trajectory(1)*sin(-yaw_d);
        		y = trajectory(1)*cos(-yaw_d) - trajectory(0)*sin(-yaw_d);
        		trajectory << x, y, trajectory(2), yaw_d;
        		if(t_straight >= d){
        			t = 0;
        			t_straight = 0;
        			waypoint++;
        		}
        	}
        	else if(waypoint < waypoints.rows() - 2){
        		//cout << "[Trajectory]: waypoint = " << waypoint << endl;
        		w1 << waypoints.row(waypoint).transpose();
        		w2 << waypoints.row(waypoint + 1).transpose();
        		double d = distance(w1, w2);
        		trajectory << (1 - t / d) * w1 + t / d * w2, yaw_d;
        		x = trajectory(0)*cos(-yaw_d) + trajectory(1)*sin(-yaw_d);
        		y = trajectory(1)*cos(-yaw_d) - trajectory(0)*sin(-yaw_d);
        		trajectory << x, y, trajectory(2), yaw_d;
        		if(t >= d){
        			t = 0;
        			t_straight = 0;
        			waypoint++;
        		}
        	}
        	else if(waypoint < waypoints.rows() - 1){
        		t += 0.1 * dt;
        		//cout << "[Trajectory]: waypoint = " << waypoint << endl;
        		w1 << waypoints.row(waypoint).transpose();
        		w2 << waypoints.row(waypoint + 1).transpose();
        		double d = distance(w1, w2);
        		trajectory << (1 - t / d) * w1 + t / d * w2, yaw_d;
        		x = trajectory(0)*cos(-yaw_d) + trajectory(1)*sin(-yaw_d);
        		y = trajectory(1)*cos(-yaw_d) - trajectory(0)*sin(-yaw_d);
        		trajectory << x, y, trajectory(2), yaw_d;
        		if(t >= d){
        			t = 0;
        			t_straight = 0;
        			waypoint++;
        		}
        	}
        	else
        		trajectory << waypoints.bottomRows(1).transpose(), yaw_d;
        	velocity << 0, 0, 0, 0;
        	break;
        case 7: // aggressive 8
            t = fmod(t - 0 * speed * dt / 3, 4 + 2 * M_PI * R);
            if(t < 2)
            {
                trajectory << (R - R * t), (R - R * t), 0, 0;
                velocity << -R, -R, 0, 0;
            }
            else if(t < 2 + M_PI * R)
            {
            	trajectory << (-R - sin((t - 2) / R) * R), (-cos((t - 2) / R) * R), 0, 0;
            	velocity << -cos((t - 2) / R), sin((t - 2) / R), 0, 0;
            }else if(t < 4 + M_PI * R)
            {
            	trajectory << (-R + R * (t - (2 + M_PI * R))), (R - R * (t - (2 + M_PI * R))), 0, 0;
            	velocity << R, -R, 0, 0;
            }
            else if(t < 4 + 2 * M_PI * R)
            {
            	trajectory << (R + sin((t - (4 + M_PI * R)) / R) * R), (-cos((t - (4 + M_PI * R)) / R) * R), 0, 0;
            	velocity << cos((M_PI * R - t + 4) / R), -sin((M_PI * R - t + 4) / R), 0, 0;
            }
            trajectory = 1 * trajectory;
            velocity = 1 * speed * velocity;
            R1 << cos(-M_PI / 4), -sin(-M_PI / 4), 0, sin(-M_PI / 4), cos(-M_PI / 4), 0, 0, 0, 1;
            R2 << cos(M_PI / 18), 0, sin(M_PI / 18), 0, 1, 0, -sin(M_PI / 18), 0, cos(M_PI / 18);
            trajectory.head(3) = R1 * R2 * trajectory.head(3);
            velocity.head(3) = R1 * R2 * velocity.head(3);
            trajectory(2) += pose_d(2);
            break;
        case 8: // rectangle
            t = fmod(t - (1 - 1.0/1) * speed * dt, 8);
            if(t < 2)
            {
                trajectory << 1, 1 - t, 0, 0;
                velocity << 0, -1, 0, 0;
            }
            else if(t < 4)
            {
            	trajectory << 1 - (t - 2), -1, 0, 0;
            	velocity << -1, 0, 0, 0;
            }
            else if(t < 6)
            {
            	trajectory << -1, -1 + (t - 4), 0, 0;
            	velocity << 0, 1, 0, 0;
            }
            else if(t < 8)
            {
            	trajectory << -1 + (t - 6), 1, 0, 0;
            	velocity << 1, 0, 0, 0;
            }
            trajectory = 1 * trajectory;
            velocity = speed * velocity;
            R1 << cos(M_PI / 18), 0, sin(M_PI / 18), 0, 1, 0, -sin(M_PI / 18), 0, cos(M_PI / 18);
            trajectory.head(3) = R1 * trajectory.head(3);
            velocity.head(3) = R1 * velocity.head(3);
            trajectory(2) += pose_d(2);
            break;
        case 9: // waypoints-square
            t = fmod(t - (1 - 1.0/1) * speed * dt, 8);
            
            if(t < 1)
                trajectory << -1, -1, -0.2, 0;
            else if(t < 2)
            	trajectory << -1, 0, -0.2, 0;
            else if(t < 3)
            	trajectory << -1, 1, -0.2, 0;
            else if(t < 4)
            	trajectory << 0, 1, 0, 0;
            else if(t < 5)
            	trajectory << 1, 1, 0.2, 0;
            else if(t < 6)
            	trajectory << 1, 0, 0.2, 0;
            else if(t < 7)
            	trajectory << 1, -1, 0.2, 0;
            else if(t < 8)
            	trajectory << 0, -1, 0, 0;
            trajectory = 1 * trajectory;
            velocity << 0, 0, 0, 0;
            //R1 << cos(-M_PI / 18), 0, sin(-M_PI / 18), 0, 1, 0, -sin(-M_PI / 18), 0, cos(-M_PI / 18);
            //trajectory.head(3) = R1 * trajectory.head(3);
            trajectory(2) += pose_d(2);
            trajectory(3) = yaw_d;
            break;
        case 10: // straight line
            if(t < 100){
                trajectory << t * cos(yaw_d), t * sin(yaw_d), pose_d(2), yaw_d;
                velocity << speed * cos(yaw_d), speed * sin(yaw_d), 0, 0;
                //trajectory << t, 0, pose_d(2), yaw_d;
                //velocity << speed, 0, 0, 0;
            }
            break;
        case 11: // hover at pos
            trajectory << pose_act(0), pose_act(1), pose_d(2), yaw_d;
            velocity << 0, 0, 0, 0;
            break;
        case 12: // circle with variable speed
            t -= speed * dt;
            t += dt;
            if(fmod(t, 2 * M_PI) < M_PI / 2)
                var_speed += 1.0 / (M_PI / 2) * dt;
            else if(fmod(t, 2 * M_PI) < M_PI)
                var_speed = 1.0;
            else if(fmod(t, 2 * M_PI) < 3 * M_PI / 2)
                var_speed += 1.0 / (M_PI / 2) * dt;
            else
                var_speed -= 2.0 / (M_PI / 2) * dt;
            d += var_speed * dt;

            yaw_d = fmod(d, 2 * M_PI) - M_PI;

            trajectory << 2 * cos(d), 2 * sin(d), pose_d(2), yaw_d;
            velocity << -2 * var_speed * sin(d), 2 * var_speed * cos(d), 0, 0;
            break;
        }
    	
    	double v = sqrt(velocity(0) * velocity(0) + velocity(1) * velocity(1));
    	
    	// Publish the reference trajectory
    	geometry_msgs::QuaternionStamped trajectory_msg;
    	trajectory_msg.header.stamp = ros::Time::now();
    	trajectory_msg.quaternion.x = trajectory(0);
    	trajectory_msg.quaternion.y = trajectory(1);
    	trajectory_msg.quaternion.z = trajectory(2);
    	trajectory_msg.quaternion.w = trajectory(3);
    	trajectory_pub.publish(trajectory_msg);
    	
    	// Publish the corresponding reference trajectory velocity
    	geometry_msgs::QuaternionStamped velocity_msg;
    	velocity_msg.header.stamp = ros::Time::now();
    	velocity_msg.quaternion.x = velocity(0);
    	velocity_msg.quaternion.x = velocity(0);
    	velocity_msg.quaternion.x = velocity(0);
    	velocity_msg.quaternion.x = velocity(0);
    	velocity_pub.publish(velocity_msg);
    	
    	// Publish the trajectory type
    	traj_type.data = trajectory_type;
    	trajectory_type_pub.publish(traj_type);
    }
}

/*
 * Function to calculate distance between two waypoints
 */
double Trajectory::distance(Eigen::Vector3d v1, Eigen::Vector3d v2){
    return sqrt(pow(v1(0) - v2(0), 2) + pow(v1(1) - v2(1), 2) + pow(v1(2) - v2(2), 2));
}

int main(int argc, char** argv){
    std::cout << "[Trajectory] Trajectory generator is running..." << std::endl;

    Trajectory* controller = new Trajectory(argc, argv);

    controller->run();
}