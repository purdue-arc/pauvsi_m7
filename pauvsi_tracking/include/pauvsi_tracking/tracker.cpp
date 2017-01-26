/*
 * tracker.cpp
 *
 *  Created On: Nov 9, 2016
 *  	Author: Logesh Roshan Ramadoss
 *
 */

#include "tracker.hpp"

using std::vector;
using cv::Mat;
using cv::Point;
using cv::Vec4i;
using cv::Moments;
using cv::Point2f;
using cv::Scalar;

Tracker::Tracker()
{
	this->readROSParameters();

	image_transport::ImageTransport it(nh);

	this->cameraSub = it.subscribeCamera(this->getCameraTopic(), 1, &Tracker::cameraCallback, this);

	ROS_DEBUG_STREAM(cameraSub.getInfoTopic());
	ROS_DEBUG_STREAM(cameraSub.getTopic());
	ROS_DEBUG_STREAM(cameraSub.getTransport());

	//current Values
	ILOWHUE = 1; //maybe 0
	IHIGHHUE = 6;
	ILOWSATURATION = 170;
	IHIGHSATURATION = 228;
	ILOWVALUE = 210;
	IHIGHVALUE = 256;


	ROS_DEBUG_STREAM("Done");

}


void Tracker::cameraCallback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& cam)
{
//	this->createTrackBars();
	ROS_DEBUG_STREAM("Listening To Camera: In callback");
	//set the K and D matrices
	this->setK(get3x3FromVector(cam->K));
	this->setD(cv::Mat(cam->D, false));


	//What is this? this->setK(get3x3FromVector(cam->K));
	this->inputImg = cv_bridge::toCvShare(img, "bgr8")->image.clone();

//	cv::imshow("input", this->inputImg);
//	cv::waitKey(30);

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
	ROS_DEBUG_STREAM("camera topic is: " << cameraTopic);

	ros::param::param<std::string>("~camera_frame_name", camera_frame, DEFAULT_CAMERA_FRAME_NAME);
	ros::param::param<std::string>("~odom_frame_name", odom_frame, DEFAULT_ODOM_FRAME_NAME);
	ros::param::param<std::string>("~world_frame_name", world_frame, DEFAULT_WORLD_FRAME_NAME);
}

void Tracker::displayTargets()
{
		cv::Mat display = this->inputImg;
		for(int i=0; i<roombaPoses.size(); ++i)
		{
			cv::circle(display,Point(roombaPoses[i].x,roombaPoses[i].y),20,Scalar(0,255,0),2);
			if(roombaPoses[i].y-25>0)
				cv::line(display,Point(roombaPoses[i].x,roombaPoses[i].y),
						Point(roombaPoses[i].x,roombaPoses[i].y-25),Scalar(0,255,0),2);
			else
				line(display,Point(roombaPoses[i].x,roombaPoses[i].y),
		    		  Point(roombaPoses[i].x,0),Scalar(0,255,0),2);

			if(roombaPoses[i].y+25<display.rows)
				cv::line(display,Point(roombaPoses[i].x,roombaPoses[i].y),
						Point(roombaPoses[i].x,roombaPoses[i].y-25),Scalar(0,255,0),2);
			else
				cv::line(display,Point(roombaPoses[i].x,roombaPoses[i].y),
						Point(roombaPoses[i].x,display.rows),Scalar(0,255,0),2);

			if(roombaPoses[i].x-25>0)
				cv::line(display,Point(roombaPoses[i].x,roombaPoses[i].y),
						Point(roombaPoses[i].x-25,roombaPoses[i].y),Scalar(0,255,0),2);
			else
				cv::line(display,Point(roombaPoses[i].x,roombaPoses[i].y),
						Point(0,roombaPoses[i].y),Scalar(0,255,0),2);

			if(roombaPoses[i].x+25<display.cols)
				    cv::line(display,Point(roombaPoses[i].x,roombaPoses[i].y),
				    		Point(roombaPoses[i].x+25,roombaPoses[i].y),Scalar(0,255,0),2);
			else
				cv::line(display,Point(roombaPoses[i].x,roombaPoses[i].y),
						Point(display.cols,roombaPoses[i].y),Scalar(0,255,0),2);
		}

		roombaPoses.clear();

	    cv::imshow("RoombaPoses Targets", display);
	    cv::waitKey(1000);




}

void Tracker::createTrackBars()
{
	const std::string trackbarWindowName = "Trackbars";

    cv::namedWindow(trackbarWindowName, CV_WINDOW_AUTOSIZE);

    cv::waitKey(30);
	//create memory to store trackbar name on window
	char TrackbarName[50];
	std::sprintf( TrackbarName, "H_MIN", ILOWHUE);
	std::sprintf( TrackbarName, "H_MAX", IHIGHHUE);
	std::sprintf( TrackbarName, "S_MIN", ILOWSATURATION);
	std::sprintf( TrackbarName, "S_MAX", IHIGHSATURATION);
	std::sprintf( TrackbarName, "V_MIN", ILOWVALUE);
	std::sprintf( TrackbarName, "V_MAX", IHIGHVALUE);


    cv::createTrackbar( "H_MIN", trackbarWindowName, &ILOWHUE, IHIGHHUE);
    cv::createTrackbar( "H_MAX", trackbarWindowName, &IHIGHHUE, IHIGHHUE);
    cv::createTrackbar( "S_MIN", trackbarWindowName, &ILOWSATURATION, IHIGHSATURATION);
    cv::createTrackbar( "S_MAX", trackbarWindowName, &IHIGHSATURATION, IHIGHSATURATION);
    cv::createTrackbar( "V_MIN", trackbarWindowName, &ILOWVALUE, IHIGHVALUE);
    cv::createTrackbar( "V_MAX", trackbarWindowName, &IHIGHVALUE, IHIGHVALUE);

    ROS_DEBUG_STREAM("Hit");
}

void Tracker::run()
{
	ROS_DEBUG_STREAM("iN RUN");
	Mat imgHSV;
	Mat imgThresholded;
	//TODO: Make sure input image is BGR and not RGB
	cv::cvtColor(inputImg, imgHSV, cv::COLOR_BGR2HSV);
	//TODO: GET HSV Range for the Red Roomba. (IMPORTANT!)
	cv::inRange(imgHSV, Scalar(ILOWHUE, ILOWSATURATION, ILOWVALUE),
						Scalar(IHIGHHUE, IHIGHSATURATION, IHIGHVALUE), imgThresholded);
/*	cv::imshow("Thresholded image", imgThresholded);
	cv::waitKey(30);
	return;
*/

	//Remove small objects from the foreground (Morphological Opening)
	cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
	cv::dilate(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));

	//Fill out small holes in the background (Morphological Closing)
	cv::dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
	cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );


	Mat imgCanny;
	std::vector<vector<Point> > contours;
	std::vector<Vec4i> hierarchy;

	cv::Canny(imgThresholded, imgCanny, CANNY_THRESHOLD, CANNY_THRESHOLD*2);
	cv::findContours( imgCanny, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

	//Calculate the Moments
	std::vector<cv::Moments>  oMoments(contours.size());

	for(int i=0; i<contours.size(); ++i)
	{
		oMoments[i] = cv::moments(contours[i]);
	}


	//std::vector<Point2f> roombaPos(contours.size());
	//roombaPoses.resize(contours.size());

	for(int i=0; i<contours.size(); ++i)
	{
		//Ensure it's a roomba. Might be unnecessary
		if(oMoments[i].m00 > 1000)
		{
			roombaPoses.push_back(Point2f(oMoments[i].m10 / oMoments[i].m00 , oMoments[i].m01 / oMoments[i].m00));
			//roombaPoses[i] = ;
			ROS_WARN_STREAM("Position of"<< i+1<< "Roomba is: x:" << roombaPoses[i].x << " y:"<< roombaPoses[i].y << std::endl);
		}
	}

	displayTargets();

}

void Tracker::getWorldPosition()
{
	static tf::TransformListener listener;
	tf::StampedTransform worldToCam; // actually world to cam
	//NOTE: If you multiply WorldToCam with a pos in world frame, then you'll get a pos in cam coord frame
	try
	{
		listener.lookupTransform(this->camera_frame, this->world_frame, ros::Time(ros::Time(0)), worldToCam);
	}
	catch(tf::TransformException &e)
	{
		ROS_WARN_STREAM(e.what());
	}

	// [u v]
	vector<cv::Point2f> undistortedPoses;
	cv::fisheye::undistortPoints(roombaPoses, undistortedPoses, this->K, this->D);

	// [u v 1] format
	vector<tf::Vector3> projectedPoses;
	for(int i=0; i<undistortedPoses.size(); ++i)
	{
		projectedPoses.push_back(tf::Vector3(undistortedPoses[i].x, undistortedPoses[i].y, 1));
	}

	//[x y z] of projected points in world coordinate frame
	vector<tf::Vector3> worldProjectedPoses;
	tf::Vector3 cameraPos;

	cameraPos = worldToCam.getOrigin(); //Origin of cam in world frame

	tf::Transform camToWorld = worldToCam.inverse();

	for(auto e : projectedPoses)
	{
		worldProjectedPoses.push_back(camToWorld * (tf::Vector3(e))); // will give point in woorld coord
	}

	/*for(int i=0; i<projectedPoses.size(); ++i)
	{
		worldProjectedPoses.push_back(camToWorld * (tf::Vector3(projectedPoses[i]))); // will give point in woorld coord

	}
	*/
	//Positions of roombas in world Coordinate frame
	vector<tf::Vector3> worldRoombaPosition;
	//vector along line [a b c]
	tf::Vector3 lineVector;
	// parameter t for line vector
	double lineParameter;

	for(auto e: worldProjectedPoses)
	{
		//r = r. + tv
		//V = [a b c]
		lineVector = e - cameraPos;
		// t = (z-z.)/c
		lineParameter = (ROOMBA_HEIGHT - cameraPos.getZ())/lineVector.getZ();

		worldRoombaPosition.push_back(tf::Vector3(cameraPos.getX() + lineParameter*lineVector.getX(),
													cameraPos.getY() + lineParameter*lineVector.getY(),
													ROOMBA_HEIGHT));
	}

/*	for(int i=0; i<worldProjectedPoses.size(); ++i)
	{
		// [ a b c]
		lineVector = worldProjectedPoses[i] - cameraPos;
		// t = (z-zo)/c
		lineParameter = (ROOMBA_HEIGHT - cameraPos.getZ())/lineVector.getZ();


		worldRoombaPosition.push_back(tf::Vector3(cameraPos.getX() + lineParameter*lineVector.getX(),
													 cameraPos.getY() + lineParameter*lineVector.getY(),
													 ROOMBA_HEIGHT));
	}
*/


	for(int i = 0; i<worldRoombaPosition.size(); ++i)
	{
		ROS_WARN_STREAM("\n Roomba "<<i+1<<" Position: "<< worldRoombaPosition[i]);
	}


	tf::TransformBroadcaster br;
	for(int i=0; i<worldRoombaPosition.size(); ++i)
	{
		tf::Transform transform;
		transform.setOrigin(worldRoombaPosition[i]);
		tf::Quaternion q;
		q.setRPY(0,0,0);
		transform.setRotation(q);
//		char buffer[50];

//		std::sprintf(buffer, "%s Roomba %d", this->camera_frame, i);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), this->world_frame,
						std::string(this->camera_frame + " Roomba " + boost::lexical_cast<std::string>(i))));
//		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), this->world_frame,
//								buffer));
	}

}

/*
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
*/
PoseEstimate::PoseEstimate() {

	const std::string poseTopicName("/tracking/roombas");
	const uint32_t publisherQueueSize = 1;
	for (int i = 0; i < 5; i++) {
		Tracker foo;
		trackers.push_back(foo);
	}
	//this->roombaPos = nh.advertise<std::vector<GroundRobotPosition> >(poseTopicName,publisherQueueSize,true);

}



