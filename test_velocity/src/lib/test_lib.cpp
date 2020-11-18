#include <test_velocity/test_lib.hpp>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/TwistStamped.h>

#include <test_velocity/pbPlots.hpp>
#include <test_velocity/supportLib.hpp>
#include <vector>
#include <iostream> 
#include <string>
#include <sstream>
#include <ctime>

void captureGraph(std::vector<double> _xs, std::vector<double> _ys, std::string abc)
{
	RGBABitmapImageReference *imageReference = CreateRGBABitmapImageReference();

	//DrawScatterPlot(imageReference, 600, 400, &_xs, &_ys);
	DrawScatterPlot(imageReference, 1280, 720, &_xs, &_ys);

	vector<double> *pngdata = ConvertToPNG(imageReference->image);
	WriteToFile(pngdata, abc);
	DeleteImage(imageReference->image);
}

std::string getName()
{
	time_t now = time(0);
	tm *ltm = localtime(&now);

    std::stringstream ss[6];

    ss[0] << ltm->tm_hour;
    ss[1] << ltm->tm_min;
    ss[2] << ltm->tm_sec;
    ss[3] << ltm->tm_mday;
    ss[4] << 1 + ltm->tm_mon;
    ss[5] << 1900 + ltm->tm_year;

    std::string str1 = ss[0].str() + ss[1].str() + ss[2].str() + ss[3].str() + ss[4].str() + ss[5].str();
	std::cout << "Time : " << ss[0].str() << ":" << ss[1].str() << ":" << ss[2].str() << ", " << ss[3].str() << " " << ss[4].str() << " " <<  ss[5].str() << std::endl;
	return("graph_" + str1 +".png");
}

//check position
bool position_distance(geometry_msgs::PoseStamped e1, geometry_msgs::PoseStamped e2)
{
	Eigen::Vector3d ps1, ps2;
	tf::pointMsgToEigen(e1.pose.position, ps2);
	tf::pointMsgToEigen(e2.pose.position, ps1);
	
	// get threshold [0-1] = 6.95113e-310
	/*
     * std::array<double, 10> threshold;
     * double err_th = threshold[rand() % threshold.size()];
     */
    
	double threshold = 0.1;       //0.01 m || 0.1 m
	ROS_DEBUG("Next setpoint: accepted error threshold: %1.3f", threshold);
	double distance = sqrt( (ps2 - ps1).x() * (ps2 - ps1).x() +
							(ps2 - ps1).y() * (ps2 - ps1).y() + 
							(ps2 - ps1).z() * (ps2 - ps1).z());
	
	if (distance <= threshold){
		//ROS_INFO("The goal was successful.");
		return true;
	}
	else{
		//ROS_WARN("Can not go to the goal.");
		//ROS_WARN("Differnce distance between current position and goal posisiton is %f", distance);
		return false;
	}
}

double distance(geometry_msgs::PoseStamped e1, geometry_msgs::PoseStamped e2)
{
	Eigen::Vector3d ps1, ps2;
	tf::pointMsgToEigen(e1.pose.position, ps2);
	tf::pointMsgToEigen(e2.pose.position, ps1);

	return(sqrt((ps2 - ps1).x() * (ps2 - ps1).x() +
				(ps2 - ps1).y() * (ps2 - ps1).y() + 
				(ps2 - ps1).z() * (ps2 - ps1).z()));
}

geometry_msgs::Twist compute_velocity(geometry_msgs::PoseStamped e1, geometry_msgs::PoseStamped e2, double timer1)
{
	Eigen::Vector3d ps1, ps2;
	tf::pointMsgToEigen(e1.pose.position, ps1);
	tf::pointMsgToEigen(e2.pose.position, ps2);
	
	std::cout << "Estimated flight time: ";
	timer1 = 5;
	std::cout << timer1 << std::endl;
	//std::cin >> timer1;
	
	geometry_msgs::Twist vel_calc;
	vel_calc.linear.x = 0;
	vel_calc.linear.y = 0;
	vel_calc.linear.z = 0;
	vel_calc.angular.x = 0;
	vel_calc.angular.y = 0;
	vel_calc.angular.z = 0;
	tf::vectorEigenToMsg(Eigen::Vector3d((ps2 - ps1).x()/timer1, (ps2 - ps1).y()/timer1, (ps2 - ps1).z()/timer1), vel_calc.linear);
	ROS_INFO("The speed of the velocity is (% f, %f, %f) m/s", vel_calc.linear.x, vel_calc.linear.y, vel_calc.linear.z);
	return vel_calc;
}

/*
void captureGraph(vector<double> _xs, vector<double> _ys)
{
	RGBABitmapImageReference *imageReference = CreateRGBABitmapImageReference();

	//DrawScatterPlot(imageReference, 600, 400, &xs, &ys);
	DrawScatterPlot(imageReference, 1280, 720, &_xs, &_ys);

	vector<double> *pngdata = ConvertToPNG(imageReference->image);
	std::string test_vel_img;
	WriteToFile(pngdata, "example2.png");
	DeleteImage(imageReference->image);
}
*/

/*
void captureGraph(std::vector<double> _xs, std::vector<double> _ys, std::string abc)
{
	RGBABitmapImageReference *imageReference = CreateRGBABitmapImageReference();

	ScatterPlotSeries *series = GetDefaultScatterPlotSeriesSettings();
	series->xs = &_xs;
	series->ys = &_ys;
	series->linearInterpolation = true;
	series->lineType = toVector(L"solid");
	series->lineThickness = 2;
	series->color = GetGray(1);

	ScatterPlotSettings *settings = GetDefaultScatterPlotSettings();
	settings->width = 1280;
	settings->height = 720;
	settings->autoBoundaries = true;
	settings->autoPadding = true;
	settings->title = toVector(L"Do sai lech tu Drone den diem Dich dưa theo so vong lap");
	settings->xLabel = toVector(L"X axis");
	settings->yLabel = toVector(L"Y axis");
	settings->scatterPlotSeries->push_back(series);

	DrawScatterPlotFromSettings(imageReference, settings);

	vector<double> *pngdata = ConvertToPNG(imageReference->image);
	WriteToFile(pngdata, "hi.png");
	DeleteImage(imageReference->image);
}
*/
