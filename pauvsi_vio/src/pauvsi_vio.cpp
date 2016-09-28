#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "pauvsi_vio/vio.h"

VIO vio; // create an instance of the visual odometry algorithm

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	ros::Time start = ros::Time::now();
	cv::Mat temp = cv_bridge::toCvShare(msg, "mono8")->image.clone();
	vio.setCurrentFrame(temp, cv_bridge::toCvCopy(msg, "mono8")->header.stamp); //set the current frame and its time created

	//vio.viewImage(vio.getCurrentFrame().image, vio.getCurrentFrame().corners);
	//if(vio.isLastFrameSet()){vio.viewImage(vio.getLastFrame(), vio.getCurrentFrame(), vio.matchFeaturesWithFlann(vio.getLastFrame().descriptors, vio.getCurrentFrame().descriptors));}

	ROS_DEBUG_STREAM("message #" << cv_bridge::toCvShare(msg, "mono8")->header.seq << " finished in " << (ros::Time::now().toSec() - start.toSec()) * 1000 << " milliseconds");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pauvsi_vio", ros::init_options::AnonymousName); // initializes with a randomish name
	ros::NodeHandle nh;

	vio.readROSParameters(); // read parameters into vio class from parameter server

	//set up image transport
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber imageSub;
	imageSub = it.subscribe(vio.getCameraTopic(), 1, imageCallback);

	ros::spin();

	return 0;
}
