
#include "rplidar.hpp"

ObstacleDetector::ObstacleDetector()
{
//	this->readROSParameters();
	this->lidarSub = nh.subscribe(DEFAULT_SCAN_TOPIC, 1, &ObstacleDetector::run,this);
//	this->run();

}

void ObstacleDetector::run(const sensor_msgs::LaserScanConstPtr& input)
{
	static tf::TransformListener listener;
	tf::StampedTransform worldToLidar;
	try
		{
			listener.lookupTransform(this->lidar_frame, this->world_frame, ros::Time(ros::Time(0)), worldToLidar);
		}
		catch(tf::TransformException &e)
		{
			ROS_WARN_STREAM(e.what());
		}
	//NOTE: If you multiply WorldToCam with a pos in world frame, then you'll get a pos in cam coord frame

}
