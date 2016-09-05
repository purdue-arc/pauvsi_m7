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

void colorImageCallback(const sensor_msgs::ImageConstPtr& msg, int camNumber)
{
	ROS_DEBUG("RUNNING IMAGE CALLBACK WITH camNumber = %i", camNumber);
	cv_bridge::CvImage imageMsg = cv_bridge::toCvCopy(msg, "bgr8")->CvImage();
	imageTimes.at(camNumber) = imageMsg.header.stamp;
	ROS_DEBUG("Set image1 stamp time to %f", imageTimes.at(camNumber).toSec());
	colorImages.at(camNumber) = imageMsg.image;
	//convert the image to mono
	cvtColor(colorImages.at(camNumber), monoImages.at(camNumber), CV_BGR2GRAY);
}

void getParameters()
{
	ros::param::param<int>("~cameraTopic", numberOfCameras, 0);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gridBasedLocalizer", ros::init_options::AnonymousName); // initializes with a randomish name
	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh); // create the image transport for subsciption

	//create the numberOfCameras image subscribers
	std::vector<image_transport::Subscriber> colorImageSubs(numberOfCameras);

	//I use boost::bind(callback, _1, int) to make the number of cams dynamic
	//this sets up n callbacks with a seperate number
	for(int i = 0; i < numberOfCameras; i++)
	{
		colorImageSubs.at(i) = it.subscribe(topicNames.at(i), 1, boost::bind(colorImageCallback, _1, i));
	}

	while(nh.ok())
	{
		ros::spinOnce();
	}

	return 0;
}
