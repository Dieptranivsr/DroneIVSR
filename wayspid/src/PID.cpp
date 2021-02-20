
#include "wayspid/PID.h"

void odometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg){
    tf::Quaternion q(odometry_msg->pose.pose.orientation.x, odometry_msg->pose.pose.orientation.y, odometry_msg->pose.pose.orientation.z, odometry_msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double x, y, z;
    m.getEulerZYX(z, y, x);
    pose << odometry_msg->pose.pose.position.x, odometry_msg->pose.pose.position.y, odometry_msg->pose.pose.position.z, z;
    velocity << odometry_msg->twist.twist.linear.x, odometry_msg->twist.twist.linear.y, odometry_msg->twist.twist.linear.z, odometry_msg->twist.twist.angular.z;

    new_odometry = true;
}

void trajectoryCallback(const geometry_msgs::QuaternionStamped& trajectory_msg){
    pose_d << trajectory_msg.quaternion.x, trajectory_msg.quaternion.y, trajectory_msg.quaternion.z, trajectory_msg.quaternion.w;
}

void trajectoryVelocityCallback(const geometry_msgs::QuaternionStamped& velocity_msg){
    velocity_d << velocity_msg.quaternion.x, velocity_msg.quaternion.y, velocity_msg.quaternion.z, velocity_msg.quaternion.w;
}

void dynamicReconfigureCallback(wayspid::setPIDConfig &config, uint32_t level){
	std::cout << "[PID] setPIDConfig is running..." << std::endl;
    k_p_xy = config.k_p_xy;
    k_i_xy = config.k_i_xy;
    k_d_xy = config.k_d_xy;
    k_p_z = config.k_p_z;
    k_i_z = config.k_i_z;
    k_d_z = config.k_d_z;
}

// Constructor
PID::PID(int argc, char** argv){
    ros::init(argc, argv, "PID");
    ros::NodeHandle node_handle;

    // Subscribers
    odometry_sub = node_handle.subscribe("/uav/odometry", 1, odometryCallback);
    trajectory_sub = node_handle.subscribe("/uav/trajectory", 1, trajectoryCallback);
    trajectory_velocity_sub = node_handle.subscribe("/uav/trajectory_velocity", 1, trajectoryVelocityCallback);

    // Publishers
    velocity_pub = node_handle.advertise<geometry_msgs::Quaternion>("/uav/command_velocity", 1);

    pose << 0, 0, 0, 0;
    pose_d << 0, 0, 0, 0;
    velocity << 0, 0, 0, 0;
    velocity_d << 0, 0, 0, 0;

    if(argc == 7){
        k_p_xy = atof(argv[1]);
        k_i_xy = atof(argv[2]);
        k_d_xy = atof(argv[3]);
        k_p_z = atof(argv[4]);
        k_i_z = atof(argv[5]);
        k_d_z = atof(argv[6]);
    }
    else{
        k_p_xy = 2.0;
        k_i_xy = 0.1;
        k_d_xy = 0.6;
        k_p_z = 2.0;
        k_i_z = 0.1;
        k_d_z = 0.6;
    }

    error_i << 0, 0, 0, 0;

    //new_odometry = false;
}

// Destructor
PID::~PID(){
    ros::shutdown();
    exit(0);
}

// Function to denormalize the angle between -pi and pi
double PID::denormalizeAngle(double a1, double a2){
    if(abs(a2 - a1) > M_PI){
        if(a2 < a1)
            a1 -= 2 * M_PI;
        else
            a1 += 2 * M_PI;
    }
    return a1;
}

void PID::run(){

    // Setup dynamic reconfigure (GUI) connections
    dynamic_reconfigure::Server<wayspid::setPIDConfig> server;
    dynamic_reconfigure::Server<wayspid::setPIDConfig>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    double dt = (double)1/100;
    ros::Rate rate(100);
    double time = 0;
    int c = 0;
    geometry_msgs::Quaternion velocity_msg;

    int wayspid_type;

    // Main loop
    while(ros::ok()){
        rate.sleep();
        ros::spinOnce();

        ros::param::get("/safe_d9/controller", wayspid_type);

        if(wayspid_type != 0 && new_odometry){ // command
            pose(3) = denormalizeAngle(pose(3), pose_d(3));

            error = pose_d - pose;

            error_i += error * dt;
            error_d = velocity_d - velocity;

            // PID part
            velocity_msg.x = k_p_xy * error(0) + k_i_xy * error_i(0) + k_d_xy * error_d(0);
            velocity_msg.y = k_p_xy * error(1) + k_i_xy * error_i(1) + k_d_xy * error_d(1);
            velocity_msg.z = k_p_z * error(2) + k_i_z * error_i(2) + k_d_z * error_d(2);
            
            velocity_msg.w = 0.5 * error(3);

            velocity_pub.publish(velocity_msg);
        }
    }
}

int main(int argc, char** argv){
    std::cout << "[PID] PID position controller is running..." << std::endl;

    PID* controller = new PID(argc, argv);

    controller->run();
}
