/*
 * vio.cpp
 *
 *  Created on: Sep 19, 2016
 *      Author: kevinsheridan
 */

#include "vio.h"

VIO::VIO()
{

}

/*
 * shows cv::Mat
 */
void VIO::viewImage(cv::Mat img){
	cv::imshow("test", img);
	cv::waitKey(30);
}

void VIO::viewImage(cv::Mat img, std::vector<cv::KeyPoint> keypoints){
	cv::drawKeypoints(img, keypoints, img);
	this->viewImage(img);
}

/*
 * corrects the drift in position
 */
void VIO::correctPosition(std::vector<double> pos)
{
	position = pos;
}

/*
 * sets the current frame and starts keyframe update
 */
void VIO::setCurrentFrame(cv::Mat frame, ros::Time t)
{
	//ROS_DEBUG("setting frame");
	currentFrame = Frame(frame, t);
	currentFrame.corners = this->computeFASTFeatures(currentFrame.image, this->fastThreshold);
	currentFrame.descriptors = this->extractBRIEFDescriptors(currentFrame.image, currentFrame.corners);
}

/*
 * gets parameters from ROS param server
 */
void VIO::readROSParameters()
{
	//CAMERA TOPIC
	ROS_WARN_COND(!ros::param::has("~cameraTopic"), "Parameter for 'cameraTopic' has not been set");
	ros::param::param<std::string>("~cameraTopic", cameraTopic, DEFAULT_CAMERA_TOPIC);
	ROS_DEBUG_STREAM("camera topic is: " << cameraTopic);

	//IMU TOPIC
	ROS_WARN_COND(!ros::param::has("~imuTopic"), "Parameter for 'imuTopic' has not been set");
	ros::param::param<std::string>("~imuTopic", imuTopic, DEFAULT_IMU_TOPIC);
	ROS_DEBUG_STREAM("IMU topic is: " << imuTopic);

	ros::param::param<int>("~fast_threshold", fastThreshold, DEFAULT_FAST_THRESHOLD);
}

/*
 * finds the features within an image
 */
std::vector<cv::KeyPoint> VIO::computeFASTFeatures(cv::Mat img, int threshold){
	std::vector<cv::KeyPoint> corners;
	cv::FAST(img, corners, threshold, true); // detect with nonmax suppression
	return corners;
}

/*
 * uses the FREAK algorithm to extract feature descriptors
 */
cv::Mat VIO::extractFREAKDescriptors(cv::Mat img, std::vector<cv::KeyPoint> corners){
	cv::Ptr<cv::xfeatures2d::FREAK> extractor = cv::xfeatures2d::FREAK::create();
	cv::Mat descriptors;
	extractor->compute(img, corners, descriptors);
	//ROS_DEBUG_STREAM_THROTTLE(2, "descriptor size: " << descriptors.cols << " X " << descriptors.rows);
	return descriptors;
}

/*
 * uses the BRIEF algorithm to extract feature descriptors
 * This is the faster than FREAK
 */
cv::Mat VIO::extractBRIEFDescriptors(cv::Mat img, std::vector<cv::KeyPoint> corners){
	cv::Ptr<cv::xfeatures2d::BriefDescriptorExtractor> extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
	cv::Mat descriptors;
	extractor->compute(img, corners, descriptors);
	ROS_DEBUG_STREAM_THROTTLE(2, "descriptor size: " << descriptors.cols << " X " << descriptors.rows);
	return descriptors;
}


