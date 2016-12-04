#include<ros/ros.h>
#include "../include/pauvsi_tracking/tracker.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pauvsi_tracking", ros::init_options::AnonymousName);
	Tracker track;
	
	ros::spin();
	return 0;
}


