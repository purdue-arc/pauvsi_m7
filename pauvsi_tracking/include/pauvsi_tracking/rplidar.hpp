

#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <vector>
#include <string.h>
#include <stdio.h>

#define FREQUENCY 6.25
#define DEFAULT_SCAN_TOPIC "/scan"
#define DEFAULT_ODOM_FRAME_NAME "odom"
#define DEFAULT_COM_FRAME_NAME "base_link"
#define DEFAULT_WORLD_FRAME_NAME "world"
#define DEFAULT_RPLIDAR_FRAME_NAME "rplidar"

class ObstacleDetector
{
public:
	std::string lidar_frame;
	std::string world_frame;

	ObstacleDetector();

	void run(const sensor_msgs::LaserScanConstPtr& input);

private:
	ros::Subscriber lidarSub;
	ros::NodeHandle nh;

};
