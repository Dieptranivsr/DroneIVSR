#include <vector>
#include <iostream> 
#include <string>
#include <sstream>
#include <ctime>

#include <ros/ros.h>

#include <test_velocity/captureGraph.h>
#include <test_velocity/pbPlots.h>
#include <test_velocity/supportLib.h>

void captureGraph(std::vector<double> _xs, std::vector<double> _ys, std::string abc)
{
	RGBABitmapImageReference *imageReference = CreateRGBABitmapImageReference();

	//DrawScatterPlot(imageReference, 600, 400, &_xs, &_ys);
	//DrawScatterPlot(imageReference, 1280, 720, &_xs, &_ys);
	//DrawScatterPlot(imageReference, 1280, 1280, &_xs, &_ys); //1&2
	DrawScatterPlot(imageReference, 1280, 720, &_xs, &_ys);

	std::vector<double> *pngdata = ConvertToPNG(imageReference->image);
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

