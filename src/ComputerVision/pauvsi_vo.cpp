#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
//my libraries
#include "pauvsi_vo/vo.h"

VO vo; // create an instance of the visual odometry algorithm

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	cv::Mat temp = cv_bridge::toCvShare(msg, "mono8")->image;
	vo.setCurrentFrame(temp, cv_bridge::toCvCopy(msg, "mono8")->header.stamp); //set the current frame and its time created
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pauvsi_vo", ros::init_options::AnonymousName); // initializes with a randomish name
	ros::NodeHandle nh;

	//set up image transport
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber imageSub;
	imageSub = it.subscribe(vo.cameraTopic, 1, imageCallback);

	while(nh.ok())
	{
		ros::spinOnce();
	}

	return 0;
}
