#include <iostream> 
#include <string>
#include <vector>
#include <sstream>
#include <ctime>

#include <ros/ros.h>

#include <control_velpid/captureGraph.h>
#include <control_velpid/pbPlots.hpp>
#include <control_velpid/supportLib.hpp>

void captureGraph(std::vector<double> _xs, std::vector<double> _ys, std::string abc)
{
	RGBABitmapImageReference *imageReference = CreateRGBABitmapImageReference();

	//DrawScatterPlot(imageReference, 600, 400, &_xs, &_ys);
	//DrawScatterPlot(imageReference, 1280, 720, &_xs, &_ys);
	//DrawScatterPlot(imageReference, 1280, 1280, &_xs, &_ys); //1&2
	DrawScatterPlot(imageReference, 1280, 360, &_xs, &_ys);

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

