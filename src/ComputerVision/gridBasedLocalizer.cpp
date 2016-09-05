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

void colorImageCallback(const sensor_msgs::ImageConstPtr& msg, int camNumber)
{
	cv_bridge::CvImage imageMsg = *cv_bridge::toCvCopy(msg, "bgr8");
	imageTimes.at(camNumber) = imageMsg.header.stamp;
	ROS_DEBUG("Set image1 stamp time to %f", imageTimes.at(camNumber).toSec());
	colorImages.at(camNumber) = imageMsg.image;
}

void monoImageCallback(const sensor_msgs::ImageConstPtr& msg, int camNumber)
{
	cv_bridge::CvImage imageMsg = *cv_bridge::toCvCopy(msg, "mono8");
	imageTimes.at(camNumber) = imageMsg.header.stamp;
	ROS_DEBUG("Set image1 stamp time to %f", imageTimes.at(camNumber).toSec());
	monoImages.at(camNumber) = imageMsg.image;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gridBasedLocalizer", ros::init_options::AnonymousName); // initializes with a randomish name
	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh); // create the image transport for subsciption

	//create the MAX_CAMS * 2 image subscribers
	std::vector<image_transport::Subscriber> colorImageSubs(MAX_CAMS);
	std::vector<image_transport::Subscriber> monoImageSubs(MAX_CAMS);

	//I use boost::bind(callback, _1, int) to make the number of cams dynamic
	for

	return 0;
}
