#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "pauvsi_vo", ros::init_options::AnonymousName); // initializes with a randomish name
	ros::NodeHandle nh;
	return 0;
}
