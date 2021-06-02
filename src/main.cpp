/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

//#include "render/render.h"
#include <ctime>

#include "highway.h"


std::string getTimeStr(){
    std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::string s(30, '\0');
    std::strftime(&s[0], s.size(), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
    return s;
}

int main(int argc, char** argv)
{

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);

	// set camera position and angle
	viewer->initCameraParameters();
	float x_pos = 0;
	viewer->setCameraPosition ( x_pos-26, 0, 15.0, x_pos+25, 0, 0, 0, 0, 1);

	Highway highway(viewer);

	//initHighway(viewer);

	int frame_per_sec = 30;
	int sec_interval = 10;
	int frame_count = 0;
	int time_us = 0;

	double egoVelocity = 25;

	while (frame_count < (frame_per_sec*sec_interval))
	{
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();

		//stepHighway(egoVelocity,time_us, frame_per_sec, viewer);
		highway.stepHighway(egoVelocity,time_us, frame_per_sec, viewer);
		viewer->spinOnce(1000/frame_per_sec);
		frame_count++;
		time_us = 1000000*frame_count/frame_per_sec;
	}

	// Save NIS
	time_t now = time(0);
	tm *t_ = localtime(&now);
	std::stringstream runDirectory;
	auto startTime = std::chrono::steady_clock::now();
	runDirectory << "./run-" << 1 + t_->tm_mon << "-" << t_->tm_mday << "-" << t_->tm_hour << ":" << t_->tm_min << ":" << t_->tm_sec;
	highway.saveNISOutput(runDirectory.str());

}