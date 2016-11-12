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

const int thresh = 100;

void Tracker::cameraCallback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& cam)
{

	//What is this? this->setK(get3x3FromVector(cam->K));
	this->inputImg = cv_bridge::toCvShare(img, "mono8")->image.clone();

	vector<GroundRobotPosition> results; 

	Mat canny_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	/// Detect edges using canny
	Canny( this->inputImg, canny_output, thresh, thresh*2, 3 );
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

