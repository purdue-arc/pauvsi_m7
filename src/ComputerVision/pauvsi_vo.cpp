#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
//my libraries
#include "pauvsi_vo/vo.h"

VO vo(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	//add images
	cv::Mat temp = cv_bridge::toCvShare(msg, "bgr8")->image;
	temp.copyTo(temp);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pauvsi_vo", ros::init_options::AnonymousName); // initializes with a randomish name
	ros::NodeHandle nh;

	//set up image transport
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber imageSub;

	while(nh.ok())
	{
		ros::spinOnce();
	}

	return 0;
}
