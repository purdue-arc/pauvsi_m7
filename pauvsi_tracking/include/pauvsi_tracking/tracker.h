/*
 * tracker.h
 *
 *  Created on: Nov 4, 2016
 *	Author: Logesh Roshan
*/

#ifndef PAUVSI_TRACKER_INCLUDE_TRACKER_H_
#define PAUVSI_TRACKER_INCLUDE_TRACKER_H_

#include <ros/ros.h>
#include <ros/publisher.h>

#include "opencv2/core/core.hpp"
#include "opencv2/video.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <stdio.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <vector>


#define DEFAULT_CAMERA_TOPIC "/camera/image"


class Tracker
{
 public:

	Tracker(void);
	~Tracker(void);

	void cameraCallback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& cam);
	void readROSParameters();

	std::string getCameraTopic(){
			return cameraTopic;
	}

 protected:

	ros::NodeHandle nh;
	image_transport::CameraSubscriber cameraSub;
	ros::Publisher roombaPos;
	cv::Mat inputImg;
	std::string cameraTopic;

};









#endif
