#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "math.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>


#define LINETHRESHOLD 10 // The minimum distance of two points to be considered a line.

void associateLines(std::vector<cv::Vec2f> detectedLines);
double getDistance(int x1, int y1, int x2, int y2);

struct Line { 
	cv::Point startPoint;
	cv::Point endPoint;
	int health;
	struct Quad{
		double pitch;
		double roll;
		double yaw;
		double X;
		double Y;
	};
};



cv::Mat frame;//Current Picture
cv::Mat prevFrame;//Previous Picture


void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  frame = cv_ptr->image;

}
std::vector<Line> lines;

//Constants


//Put in arguments for when you start the node
int main(int argc, char **argv) {

  // initialize rose node with "name"
  ros::init(argc, argv, "posFind");
  // Creates Ross node handle
  ros::NodeHandle n;

  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

  image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this); //Reads images from other program


  for (ros::Rate loop_rate(10); ros::ok(); loop_rate.sleep()) {
      ros::spinOnce();


      // Get components
      std::vector<cv::Mat> channels;
      split(frame, channels);
      std::string names[] = {"B","G","R"};
      cv::Mat lineMat = frame.clone();
      cv::Mat circleMat = frame.clone();

      for (int i=0; i < 3; i++){
          cv::Mat& channel = channels[i];
          cv::Mat edges;
          //bilateralFilter (channel, output, 7, 7, 7);
          GaussianBlur(channel, channel, cv::Size(5,5), 0);
          Canny (channel, edges,  1, 255);
          add(channel, edges, channel);
          //Laplacian(output, channel, channel.depth(), 5);
          //cv::threshold (channel, output, 254, 255, CV_THRESH_BINARY);
          //channel = output;
          //imshow(names[i], edges);
          std::vector<cv::Vec2f> initial_lines;
          HoughLines(edges, initial_lines, 1, CV_PI/180, 100, 0, 0 );
          associateLines(initial_lines);
          //          for( size_t i = 0; i < initial_lines.size(); i++ ) {
          //              float &rho = initial_lines[i][0], &theta = initial_lines[i][1];
          //              cv::Point pt1;
          //              cv::Point pt2;
          //              double a = cos(theta), b = sin(theta);
          //              double x0 = a*rho, y0 = b*rho;
          //              pt1.x = cvRound(x0 + 1000*(-b));
          //              pt1.y = cvRound(y0 + 1000*(a));
          //              pt2.x = cvRound(x0 - 1000*(-b));
          //              pt2.y = cvRound(y0 - 1000*(a));
          //              line( lineMat, pt1, pt2, 0, 1, CV_AA);
          //            }

        }
      merge (channels, frame);
      imshow( "#selfie", frame);
      //imshow( "#selfieLines", lineMat);
      if (cv::waitKey(1) >= 0)
        break;
    }
  return 0;

	// initialize rose node with "name"
	ros::init(argc, argv, "talker");
	// Creates Ross node handle
	ros::NodeHandle n;
	//0 is the id of video device.0 if you have only one frame.
	cv::VideoCapture webcam(0);

	//check if video device has been initialised
	if (!webcam.isOpened()) {
		ROS_INFO("cannot open camera :-(");
	}
	ROS_INFO("got camera");

	cv::Mat frame;
	cv::Mat prevFrame;

	for (ros::Rate loop_rate(10); ros::ok(); loop_rate.sleep()) {
		ros::spinOnce();

		webcam.read(frame);

		// Get components
		std::vector<cv::Mat> channels;
		split(frame, channels);
		std::string names[] = {"B","G","R"};
		cv::Mat lineMat = frame.clone();
		cv::Mat circleMat = frame.clone();

		for (int i=0; i < 3; i++){
			cv::Mat& channel = channels[i];
			cv::Mat edges;
			//bilateralFilter (channel, output, 7, 7, 7);
			GaussianBlur(channel, channel, cv::Size(5,5), 0);
			Canny (channel, edges,  1, 255);
			add(channel, edges, channel);
			//Laplacian(output, channel, channel.depth(), 5);
			//cv::threshold (channel, output, 254, 255, CV_THRESH_BINARY);
			//channel = output;
			//imshow(names[i], edges);
			std::vector<cv::Vec2f> initial_lines;
			HoughLines(edges, initial_lines, 1, CV_PI/180, 100, 0, 0 );
			associateLines(initial_lines);
			//          for( size_t i = 0; i < initial_lines.size(); i++ ) {
			//              float &rho = initial_lines[i][0], &theta = initial_lines[i][1];
			//              cv::Point pt1;
			//              cv::Point pt2;
			//              double a = cos(theta), b = sin(theta);
			//              double x0 = a*rho, y0 = b*rho;
			//              pt1.x = cvRound(x0 + 1000*(-b));
			//              pt1.y = cvRound(y0 + 1000*(a));
			//              pt2.x = cvRound(x0 - 1000*(-b));
			//              pt2.y = cvRound(y0 - 1000*(a));
			//              line( lineMat, pt1, pt2, 0, 1, CV_AA);
			//            }

		}
		merge (channels, frame);
		//imshow( "#selfie", frame);
		imshow( "#selfieLines", lineMat);
		if (cv::waitKey(1) >= 0)
			break;
	}
	return 0;

}

void associateLines(std::vector<cv::Vec2f> detectedLines){  //Compare Past Lines to Lines currently detected
	for(size_t i = 0; i < detectedLines.size(); i++){

		float &rho = detectedLines[i][0], &theta = detectedLines[i][1];
		cv::Point pt1;
		cv::Point pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a*rho, y0 = b*rho;
		pt1.x = cvRound(x0 + 1000*(-b));
		pt1.y = cvRound(y0 + 1000*(a));
		pt2.x = cvRound(x0 - 1000*(-b));
		pt2.y = cvRound(y0 - 1000*(a));

		for(size_t j = 0; j < lines.size(); j++){

		}
	}
}

double getDistance(int x1, int y1, int x2, int y2){
	return sqrt(pow((double)(x2 - x1), 2) + pow((double)(y2 - y1), 2));
}
