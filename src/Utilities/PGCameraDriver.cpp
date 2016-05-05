#include <ros/ros.h>
#include <ros/console.h>
#include <flycapture/FlyCapture2.h>

#define DEFAULT_RATE 10

FlyCapture2::Camera cam;
FlyCapture2::CameraInfo cInfo;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "CameraDriver", ros::init_options::AnonymousName); // initializes with a randomish name
	ros::NodeHandle nh; // create the node handler

	int rate;
	ros::param::param<int>("frame_rate", rate, DEFAULT_RATE);
	ros::Rate loop_rate(rate);

	//the main loop
	while(nh.ok())
	{

		//SLEEP
		loop_rate.sleep();
	}

	return 0;
}


