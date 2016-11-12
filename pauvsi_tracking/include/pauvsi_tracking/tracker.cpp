/*
 * tracker.cpp
 *
 *  Created On: Nov 9, 2016
 *  	Author: Logesh Roshan Ramadoss
 *
 */

#include "tracker.h"
#include "tracking.hpp"

using std::vector;
using cv::Mat;
using cv::Point;
using cv::Vec4i;
using cv::Moments;
using cv::Point2f;
using cv::Scalar;

Tracker::Tracker()
{
	image_transport::ImageTransport it(nh);
	this->cameraSub = it.subscribeCamera(this->getCameraTopic(), 1, &Tracker::cameraCallback, this);

}


void Tracker::cameraCallback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& cam)
{

	//What is this? this->setK(get3x3FromVector(cam->K));
	this->inputImg = cv_bridge::toCvShare(img, "mono8")->image.clone();


	this->run();

}

cv::Mat Tracker::get3x3FromVector(boost::array<double, 9> vec)
{
	cv::Mat mat = cv::Mat(3,3, CV_32F);
	for(int i=0; i<3; ++i)
	{
		mat.at<float>(i, 0) = vec.at(3 * i + 0);
		mat.at<float>(i, 1) = vec.at(3 * i + 1);
		mat.at<float>(i, 2) = vec.at(3 * i + 2);
	}

	//ROS_DEBUG_STREAM_ONCE("K = " << mat);
	return mat;
}

void Tracker::readROSParameters()
{
	//CAMERA TOPIC
	ROS_WARN_COND(!ros::param::has("~cameraTopic"), "Parameter for 'cameraTopic' has not been set");
	ros::param::param<std::string>("~cameraTopic", cameraTopic, DEFAULT_CAMERA_TOPIC);
	//ROS_DEBUG_STREAM("camera topic is: " << cameraTopic);

	ros::param::param<std::string>("~camera_frame_name", camera_frame, DEFAULT_CAMERA_FRAME_NAME);
	ros::param::param<std::string>("~odom_frame_name", odom_frame, DEFAULT_ODOM_FRAME_NAME);
	ros::param::param<std::string>("~world_frame_name", world_frame, DEFAULT_WORLD_FRAME_NAME);
}


void Tracker::run()
{
	vector<GroundRobotPosition> results; 

	Mat canny_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	/// Detect edges using canny
	Canny( this->inputImg, canny_output, CANNY_THRESHOLD, CANNY_THRESHOLD*2, 3 );
	/// Find contours
	findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	/// Get the moments
	vector<Moments> myMoments(contours.size() );
	for( int i = 0; i < contours.size(); i++ )
		myMoments[i] = moments( contours[i], false );

	///  Get the mass centers:
	vector<Point2f> massCentetrs( contours.size() );
	for( int i = 0; i < contours.size(); i++ )
		massCentetrs[i] = Point2f( myMoments[i].m10/myMoments[i].m00 , myMoments[i].m01/myMoments[i].m00 );

	/// Draw contours
	//Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
	//for( int i = 0; i< contours.size(); i++ )
	//{
		//Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		//drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
		//circle( drawing, massCentetrs[i], 4, color, -1, 8, 0 );
	//}

	/// Show in a window
	//namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
	//imshow( "Contours", drawing );

	/// Calculate the area with the moments 00 and compare with the result of the OpenCV function
	//printf("\t Info: Area and Contour Length \n");
	//for( int i = 0; i< contours.size(); i++ )
	//{
		//printf(" * Contour[%d] - Area (M_00) = %.2f - Area OpenCV: %.2f - Length: %.2f \n", i, myMoments[i].m00, contourArea(contours[i]), arcLength( contours[i], true ) );
		//Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		//drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
		//circle( drawing, massCentetrs[i], 4, color, -1, 8, 0 );
	//}

	//this->roombaPos.publish(results);

}



int main(int argc, char** argv) {

}

