#include <ros/ros.h>
#include <ros/console.h>
#include <flycapture/FlyCapture2.h>

#define DEFAULT_RATE 10

int main(int argc, char **argv)
{
	ros::init(argc, argv, "CameraDriver", ros::init_options::AnonymousName); // initializes with a randomish name
	ros::NodeHandle nh; // create the node handler

	int rate;
	ros::param::param<int>("frame_rate", rate, DEFAULT_RATE);
	ros::Rate loop_rate(rate);

	//setup camera
	FlyCapture2::Error error;
	FlyCapture2::Camera camera;
	FlyCapture2::CameraInfo camInfo;

	// Connect the camera
	{
		error = camera.Connect( 0 );
		if ( error != FlyCapture2::PGRERROR_OK )
		{
			ROS_INFO("Failed to connect to camera\n");
			return false;
		}

		// Get the camera info and print it out
		error = camera.GetCameraInfo( &camInfo );
		if ( error != FlyCapture2::PGRERROR_OK )
		{
			ROS_INFO("Failed to get camera info from camera\n");
			return false;
		}

		std::cout << "Model: " << camInfo.modelName << std::endl;

		error = camera.StartCapture();
		if ( error == FlyCapture2::PGRERROR_ISOCH_BANDWIDTH_EXCEEDED )
		{
			ROS_INFO("Bandwidth exceeded\n");
			return false;
		}
		else if ( error != FlyCapture2::PGRERROR_OK )
		{
			ROS_INFO("Failed to start image capture\n");
			return false;
		}
	}

	//the main loop
	while(nh.ok())
	{

		//SLEEP
		loop_rate.sleep();
	}

	return 0;
}


