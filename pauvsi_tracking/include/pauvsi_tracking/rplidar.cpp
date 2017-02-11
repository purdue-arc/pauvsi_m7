#include "rplidar.hpp"



ObstacleDetector::ObstacleDetector(void)
{
	//	this->readROSParameters();
		this->lidarSub = nh.subscribe(DEFAULT_SCAN_TOPIC, 1, &ObstacleDetector::lidarCallback,this);
		this->obstaclePublisher = nh.advertise<geometry_msgs::PoseArray>("obstaclePoses", 1);
	//	this->obstaclePublisher = nh.advertise<std::vector<tf::Vector3>>("obstaclePoses", 1);

	//	this->run();
}


void ObstacleDetector::lidarCallback(const sensor_msgs::LaserScanConstPtr& input)
{
	this->lidarInput = *input;
	this->testLidar();

	//this->run();
}

void ObstacleDetector::testLidar()
{
	std::vector<tf::Vector3> lidarPoses;
	lidarPoses.reserve(this->lidarInput.ranges.size());
	for(int i=0; i<this->lidarInput.ranges.size(); ++i)
	{
		lidarPoses.push_back(tf::Vector3(lidarInput.ranges.at(i)*cos(lidarInput.angle_min + lidarInput.angle_increment*i),
										 lidarInput.ranges.at(i)*sin(lidarInput.angle_min + lidarInput.angle_increment*i),
										 0));
	}


	//NOTE: Points behind rplidar. (180 degrees behind nose)
	ROS_DEBUG_STREAM("\n (x,y,z): (" << lidarPoses.at(0).getX() <<", "<< lidarPoses.at(0).getY()<<", " << lidarPoses.at(0).getZ()<<" )");
	ROS_DEBUG_STREAM("\n (x,y,z): (" << lidarPoses.at(1).getX() <<", "<< lidarPoses.at(1).getY()<<", " << lidarPoses.at(1).getZ()<<" )");
	ROS_DEBUG_STREAM("\n (x,y,z): (" << lidarPoses.at(2).getX() <<", "<< lidarPoses.at(2).getY()<<", " << lidarPoses.at(2).getZ()<<" )");
	ROS_DEBUG_STREAM("\n (x,y,z): (" << lidarPoses.at(3).getX() <<", "<< lidarPoses.at(3).getY()<<", " << lidarPoses.at(3).getZ()<<" )\n\n");
/*
	for(auto e:lidarPoses)
	{
		ROS_DEBUG_STREAM("\n (x,y,z): (" << e.getX() <<", "<< e.getY()<<", " << e.getZ()<<" )");
	}
*/
}

void ObstacleDetector::run()
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

	tf::Transform lidarToWorld = worldToLidar.inverse();

	std::vector<tf::Vector3> lidarPoses;
	lidarPoses.reserve(this->lidarInput.ranges.size());
	for(int i=0; i<this->lidarInput.ranges.size(); ++i)
	{
		lidarPoses.push_back(tf::Vector3(lidarInput.ranges.at(i)*cos(lidarInput.angle_min + lidarInput.angle_increment*i),
										 lidarInput.ranges.at(i)*sin(lidarInput.angle_min + lidarInput.angle_increment*i),
										 0));
	}

	std::vector<tf::Vector3> worldPoses;
	worldPoses.reserve(lidarPoses.size());
	for(auto e:lidarPoses)
	{
		worldPoses.push_back(lidarToWorld * e);
	}

	geometry_msgs::PoseArray obstaclePoses;
	geometry_msgs::Pose temp;
	obstaclePoses.header.stamp = ros::Time(ros::Time(0));

	//tf::Vector3 obstaclePoses[];
	//obstaclePoses.
	//std::vector<tf::Vector3> obstaclePoses;
	for(auto e:worldPoses)
	{
		if(e.getZ() >= 0.3 && e.getZ() <= 2.0 &&
		   e.getX() >= -10.0 && e.getX() <= 10.0 &&
		   e.getY() >= -10.0 && e.getY() <= 10.0)
		{
			temp.position.x = e.getX();
			temp.position.y = e.getY();
			temp.position.z = e.getZ();
			obstaclePoses.poses.push_back(temp);
//				obstaclePoses.push_back(e);
		}
	}
//TODO: Account for rotation time
//TODO: to remove copies find distance in xy plane, remove eif below threshold
// Eg: if pt 5 is no longer the same as pt 1, then pt6 will not be same as pt 1, due to rotation in angle
	this->obstaclePublisher.publish(obstaclePoses);
//	this->lidarInput.ranges.size();

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
