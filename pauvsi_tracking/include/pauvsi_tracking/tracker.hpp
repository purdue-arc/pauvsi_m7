/*
 * tracker.h
 *
 *  Created on: Nov 4, 2016
 *	Author: Logesh Roshan
*/

#ifndef PAUVSI_TRACKER_INCLUDE_TRACKER_H_
#define PAUVSI_TRACKER_INCLUDE_TRACKER_H_


#include <opencv2/calib3d.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/video.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <stdio.h>
#include <boost/lexical_cast.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
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
#include <std_msgs/Header.h>

#define CANNY_THRESHOLD 100
#define ROOMBA_HEIGHT 0.09
#define DEFAULT_CAMERA_TOPIC "/camera/image"
#define DEFAULT_ODOM_FRAME_NAME "odom"
#define DEFAULT_CAMERA_FRAME_NAME "camera_frame"
#define DEFAULT_COM_FRAME_NAME "base_link"
#define DEFAULT_WORLD_FRAME_NAME "world"


struct GroundRobotPosition {
	double x, y, dx, dy, theta;
};

class Tracker
{
 public:

	Tracker(void);
	Tracker(std::string cameraTopic, std::string cameraFrame, int ILOWHUE, int IHIGHHUE, int ILOWSATURATION, int IHIGHSATURATION,
			int ILOWVALUE, int IHIGHVALUE);

	//frames
	std::string camera_frame;
	std::string odom_frame;
	std::string world_frame;

	void cameraCallback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& cam);
	void readROSParameters();

	std::string getCameraTopic(){
			return cameraTopic;
	}

	void setK(cv::Mat _K)
	{
		K = _K;
	}

	void setD(cv::Mat _D)
	{
		D = _D;
	}

	std_msgs::Header getHeader()
	{
		return imageHeader;
	}

	void createTrackBars();

	void run();

	void getWorldPosition();

	std::vector<tf::Vector3> getPoses();

	//For visualizing
	void displayTargets();

	cv::Mat get3x3FromVector(boost::array<double, 9> vec);


 protected:

	int ILOWHUE;
	int IHIGHHUE;
	int ILOWSATURATION;
	int IHIGHSATURATION;
	int ILOWVALUE;
	int IHIGHVALUE;
	ros::NodeHandle nh;
	image_transport::CameraSubscriber cameraSub;
	std_msgs::Header imageHeader;
	ros::Publisher roombaPos;
	cv::Mat inputImg;
	std::string cameraTopic;
	std::vector<tf::Vector3> worldRoombaPosition;

	//(x,y) positions of Roombas within image
	std::vector<cv::Point2f> roombaPoses;

	//Camera Parameter (Intrinsic and distortion)
	cv::Mat K;
	cv::Mat D;
};

class PoseEstimate {
	public:
		PoseEstimate();
		~PoseEstimate();

		ros::Publisher roombaPos;
	protected:
		std::vector<Tracker> trackers;
		ros::NodeHandle nh;


};



#endif
