#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <math.h>
#include "gridLoc.h"

std::vector<ros::Time> imageTimes; // stores the time a picture was taken
std::vector<cv::Mat> monoImages; //stores the undistorted mono image
std::vector<cv::Mat> colorImages; //stores the undistorted color image

//PARAMETERS
int numberOfCameras = 0; // the number of cameras used in the system
std::vector<std::string> topicNames;
int gridWidth = 0;
int gridHeight = 0;
double gridSpacing = 0;

void colorImageCallback(const sensor_msgs::ImageConstPtr& msg, int camNumber)
{
	ROS_DEBUG_THROTTLE(1, "RUNNING IMAGE CALLBACK WITH camNumber = %i", camNumber);
	cv_bridge::CvImage imageMsg = *cv_bridge::toCvCopy(msg, "bgr8");
	imageTimes.at(camNumber) = imageMsg.header.stamp;
	ROS_DEBUG_THROTTLE(1, "Set image1 stamp time to %f", imageTimes.at(camNumber).toSec());
	colorImages.at(camNumber) = imageMsg.image;
	//convert the image to mono
	cvtColor(colorImages.at(camNumber), monoImages.at(camNumber), CV_BGR2GRAY);
}

void getParameters()
{
	ros::param::param<int>("~numberOfCams", numberOfCameras, 0);
	ROS_WARN_COND(numberOfCameras <= 0, "%i cameras are being used", numberOfCameras);
	//setup grid specs
	ros::param::param<int>("~gridWidth", gridWidth, 0);
	ros::param::param<int>("~gridHeight", gridHeight, 0);
	ros::param::param<double>("~gridSpacing", gridSpacing, 0);
	ROS_INFO("Using %i camera(s) to track a %i X %i grid with %f m spacing", numberOfCameras, gridWidth, gridHeight, gridSpacing);

	//resize the vectors for the camera size
	topicNames.resize(numberOfCameras);
	for(int i = 0; i < numberOfCameras; i++)
	{
		std::ostringstream s;

		s.clear();
		s << "~cameraTopic_" << i;
		ROS_DEBUG_STREAM("getting " << s.str() << " parameter");
		ros::param::param<std::string>(s.str(), topicNames.at(i), "none");
		ROS_DEBUG_STREAM(s.str() << " param is " << topicNames.at(i));
	}

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gridBasedLocalizer", ros::init_options::AnonymousName); // initializes with a randomish name
	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh); // create the image transport for subsciption

	//get the parameters
	getParameters();

	//create the numberOfCameras image subscribers
	std::vector<image_transport::Subscriber> colorImageSubs;
	colorImageSubs.resize(numberOfCameras);
	ROS_DEBUG("set the colorImageSubs vector to %i", (int)colorImageSubs.size());

	//set the size of the image and time vectors
	imageTimes.resize(numberOfCameras);
	monoImages.resize(numberOfCameras);
	colorImages.resize(numberOfCameras);

	ROS_DEBUG("resized the three image and time vectors");

	//I use boost::bind(callback, _1, int) to make the number of cams dynamic
	//this sets up n callbacks with a seperate number
	for(int i = 0; i < numberOfCameras; i++)
	{
		ROS_DEBUG("running through sub loop %i", i);
		ROS_DEBUG_STREAM("Setting up sub for " << topicNames.at(i));
		colorImageSubs.at(i) = it.subscribe(topicNames.at(i), 1, boost::bind(colorImageCallback, _1, i));
	}

	while(nh.ok())
	{
		ros::spinOnce();
	}

	return 0;
}
