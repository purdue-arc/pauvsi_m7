#include <ros/ros.h>
#include "../include/pauvsi_tracking/rplidar.hpp"


int main(int argc, char** argv)
{
	ros::init(argc, argv, "obstacle_tracking", ros::init_options::AnonymousName);
	ObstacleDetector tracker;

	ros::spin();
	return 0;
}
