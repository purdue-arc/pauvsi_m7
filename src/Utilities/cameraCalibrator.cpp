#include <ros/ros.h>
#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <stdlib.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv;

int n_boards = 0;  //Number of snapshots of the chessboard
int frame_step;   //Frames to be skipped
int board_w;   //Enclosed corners horizontally on the chessboard
int board_h;   //Enclosed corners vertically on the chessboard

cv::Mat imageMat;
//IplImage image;
bool imageSet = false;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

	cv::Mat temp = cv_bridge::toCvShare(msg, "bgr8")->image;

	temp.copyTo(imageMat);

	ROS_DEBUG("IMAGE COLS %i", imageMat.cols);
	ROS_DEBUG("IMAGE ROWS %i", imageMat.rows);

	//cv::imshow("test", imageMat);
	//cv::waitKey(30);

	imageSet = true;
}

cv::Mat captureFrame()
{
	ros::Rate r(100);
	while(!imageSet)
	{
		ros::spinOnce();
		r.sleep();
	}
	imageSet = false;

	//image = imageMat;
	//IplImage* image;
	//IplImage* image = cvCreateImage(cvSize(imageMat.cols, imageMat.rows), 8, 3);
	//IplImage temp = imageMat;
	//cvCopy(&temp, image);
	cv::imshow("test", imageMat);
	cv::waitKey(30);
	return imageMat;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "CameraCalibrator", ros::init_options::AnonymousName); // initializes with a randomish name
	ros::NodeHandle nh; // create the node handler

	//set up image transport
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber imageSub;

	std::string topicName;
	ros::param::param<std::string>("~cameraTopic", topicName, "unknown");
	imageSub = it.subscribe(topicName, 1, imageCallback);


	//CAMERA CALIBRATION CODE

	int numBoards = 0;
	int numCornersHor;
	int numCornersVer;

	ros::Duration wait(1);
	wait.sleep();
	ROS_INFO("Enter number of corners along width: ");
	scanf("%d", &numCornersHor);

	ROS_INFO("Enter number of corners along height: ");
	scanf("%d", &numCornersVer);

	ROS_INFO("Enter number of boards: ");
	scanf("%d", &numBoards);

	int numSquares = numCornersHor * numCornersVer;
	cv::Size board_sz = cv::Size(numCornersHor, numCornersVer);
	//cv::VideoCapture capture = cv::VideoCapture(0);

	std::vector<std::vector<cv::Point3f> > object_points;
	std::vector<std::vector<cv::Point2f> > image_points;

	std::vector<cv::Point2f> corners;
	int successes=0;

	cv::Mat image;
	cv::Mat gray_image;
	//capture >> image;

	ros::spinOnce();
	image = imageMat;
	//image = captureFrame();

	std::vector<cv::Point3f> obj;
	for(int j=0;j<numSquares;j++)
	{
		obj.push_back(Point3f(j/numCornersHor, j%numCornersHor, 0.0f));
	}

	//cv::imshow("test", imageMat);
	//cv::waitKey(30);

	while(successes<numBoards && nh.ok())
	{
		cvtColor(image, gray_image, CV_BGR2GRAY);



		bool found = cv::findChessboardCorners(image, board_sz, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

		if(found)
		{
			ROS_DEBUG("FOUND CORNERS RUNNING cornerSubPix with corner count %i", (int)corners.size());
			cornerSubPix(gray_image, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			ROS_DEBUG("DRAWING CORNERS");
			cv::drawChessboardCorners(gray_image, board_sz, corners, found);
		}

		imshow("color", image);
		imshow("Corner Tracking", gray_image);

		//image = captureFrame();
		//ros::spinOnce();
		//image = imageMat;

		int key = cv::waitKey(30);

		if(key==27)
			return 0;

		ROS_DEBUG("The Key currently pressed is %i", key);
		key=' ';
		if(key==' ' && found!=0)
		{
			image_points.push_back(corners);
			object_points.push_back(obj);
			ROS_INFO("Snap Stored...");

			successes++;

			if(successes>=numBoards)
				break;
		}
		//ros::spinOnce();
		ros::spinOnce();
		image = imageMat;
	}

	cv::Mat intrinsic = cv::Mat(3, 3, CV_32FC1);
	cv::Mat distCoeffs;
	std::vector<cv::Mat> rvecs;
	std::vector<cv::Mat> tvecs;

	intrinsic.ptr<float>(0)[0] = 1;
	intrinsic.ptr<float>(1)[1] = 1;

	cv::calibrateCamera(object_points, image_points, image.size(), intrinsic, distCoeffs, rvecs, tvecs);

	//SAVING
	ROS_INFO("Calibration done!");
	ROS_INFO("COEFFICIENTS of Intrinsic Matrix:");
	ROS_INFO_STREAM("Intrinsic = " << intrinsic << std::endl);
	ROS_INFO("COEFFICIENTS of Distortion Matrix:");
	ROS_INFO_STREAM("Distortion = " << distCoeffs << std::endl);

	ROS_DEBUG_STREAM("Dist dimensions rows: " << distCoeffs.rows << " cols: " << distCoeffs.cols);
	//imwrite("calibration/Instrinsics.xml", intrinsic);
	//imwrite("calibration/Distortion.xml", distCoeffs);
	ROS_DEBUG("SAVING AS FILE...");
	FileStorage fsi("../params/intrinsic.xml", FileStorage::WRITE);
	fsi << intrinsic;
	FileStorage fsd("../params/distortion.xml", FileStorage::WRITE);
	fsd << distCoeffs;

	fsi.release();
	fsd.release();

	cv::Mat imageUndistorted;
	while(nh.ok())
	{
		//image = captureFrame();
		ros::spinOnce();
		image = imageMat;

		cv::undistort(image, imageUndistorted, intrinsic, distCoeffs);

		cv::imshow("Raw image", image);
		cv::imshow("Undistorted image", imageUndistorted);

		cv::waitKey(1);
		ros::spinOnce();
	}

	//capture.release();
	/*while(nh.ok())
	{
		ros::spinOnce();
		//cv::imshow("win1", imageMat);
		//cv::waitKey(1);
	}*/

	return 0;
}

