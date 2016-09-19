#include <ros/ros.h>
#include <ros/console.h>
#include <flycapture/FlyCapture2.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <string>
#include <math.h>

using std::string;

int connectCamera();
void getParameters();
bool setImageSettings(int x_offset, int y_offset, int width, int height, FlyCapture2::PixelFormat pixelForm, FlyCapture2::Mode mode);
bool setProperty(const FlyCapture2::PropertyType &type, const bool &autoSet, unsigned int &valueA, unsigned int &valueB);
bool adjustCameraSettings();
cv::Mat captureAndConvert();
cv::Mat createMatFromString(std::string text);
string removeSpaces(string input);

#define DEFAULT_RATE 10

int rate;
int serial;
bool crop;
bool pubUndistorted = false;
bool pubDistorted = false;
string messagePrefix;
int cropX;
int cropY;
string distortionString;
string intrinsicString;

//distortion matrices
cv::Mat intrinsic;
cv::Mat distortion;

FlyCapture2::Error error;
FlyCapture2::Camera camera;
FlyCapture2::CameraInfo camInfo;
FlyCapture2::BusManager busMngr;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "CameraDriver", ros::init_options::AnonymousName); // initializes with a randomish name
	ros::NodeHandle nh; // create the node handler

	getParameters(); // set all params
	ROS_DEBUG("Params %i, %i, %i", rate, serial, crop);

	//CREATE THE DISTORTION MATS
	intrinsic = createMatFromString(intrinsicString);
	distortion = createMatFromString(distortionString);

	// CONNECT CAMERA
	connectCamera();

	//SETUP PUBLISHERS
	image_transport::ImageTransport it(nh);
	//image_transport::ImageTransport it3(nh);
	string topicName = "";

	image_transport::Publisher undistortedColorPub;
	topicName = "PGCameraDriver/";
	topicName += messagePrefix;
	topicName += "/color/undistorted";
	undistortedColorPub = it.advertise(topicName, 1);

	image_transport::Publisher distortedColorPub;
	topicName = "PGCameraDriver/";
	topicName += messagePrefix;
	topicName += "/color/distorted";
	distortedColorPub = it.advertise(topicName, 1);

	ROS_DEBUG("set up publishers");


	//SET THE RATE
	ros::Rate loop_rate(rate);
	//the main loop
	while(nh.ok())
	{
		//SET TIMESTAMP
		std_msgs::Header head = std_msgs::Header();
		head.stamp = ros::Time::now();

		//CAPTURE
		cv::Mat raw_image = captureAndConvert();
		//cv::imshow("test", raw_image);
		//cv::waitKey(1);

		//CROP IMAGE IF REQUESTED
		if(crop)
		{
			//crop the image
			cv::Rect roi(round(double(raw_image.cols - cropX) / 2.0), round(double(raw_image.rows - cropY) / 2.0), cropX, cropY);
			//create image from rect
			raw_image = raw_image(roi);
		}

		//cv::imshow("test", raw_image);
		//cv::waitKey(1);

		//check if a topic has subscribers then send is message
		// distorted color publisher
		if (distortedColorPub.getNumSubscribers() > 0 && pubDistorted)
		{
			//create and publish
			sensor_msgs::ImagePtr msg = cv_bridge::CvImage(head, "bgr8", raw_image).toImageMsg();
			cv::waitKey(1);
			distortedColorPub.publish(msg);
		}

		//SLEEP
		loop_rate.sleep();
	}

	//disconnect the camera
	camera.Disconnect();

	return 0;
}

cv::Mat captureAndConvert()
{
	FlyCapture2::Image rawImage;
	error = camera.RetrieveBuffer( &rawImage );
	if ( error != FlyCapture2::PGRERROR_OK )
	{
		ROS_ERROR("Capture Error");
	}

	// CONVERT TO BGR
	FlyCapture2::Image bgrImage;
	rawImage.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &bgrImage );

	// CONVERT TO MAT
	unsigned int rowBytes = (double)bgrImage.GetReceivedDataSize()/(double)bgrImage.GetRows();
	cv::Mat temp = cv::Mat(bgrImage.GetRows(), bgrImage.GetCols(), CV_8UC3, bgrImage.GetData(),rowBytes);

	//cv::imshow("test", temp);
	//cv::waitKey(1);

	ROS_DEBUG_ONCE("Image size: %i, %i", temp.cols, temp.rows);

	return temp;
}

void getParameters()
{
	//get the parameters
	//rate
	ros::param::param<int>("~frame_rate", rate, DEFAULT_RATE);
	ROS_INFO("Camera Frame Rate: %i", rate);
	//serial
	ros::param::param<int>("~serial_number", serial, 0);
	ROS_INFO("Driver using %i as serial number", serial);
	//cam type
	ros::param::param<bool>("~crop", crop, false);
	//camerPos
	ros::param::param<string>("~camera_position", messagePrefix, "unknown");
	//size to crop ptam image by
	ros::param::param<int>("~crop_x", cropX, 1000);
	ros::param::param<int>("~crop_y", cropY, 1000);
	ROS_INFO("Cropping image to %i X %i", cropX, cropY);

	ros::param::param<bool>("~publishDistorted", pubDistorted, false);
	ros::param::param<bool>("~publishUndistorted", pubUndistorted, true);

	ros::param::param<std::string>("~cameraIntrinsic", intrinsicString, "0");
	ros::param::param<std::string>("~cameraDistortion", distortionString, "0");
}

int connectCamera()
{
	FlyCapture2::PGRGuid guid; //id of the camera
	//get the camera with specific serial
	error = busMngr.GetCameraFromSerialNumber(serial, &guid);
	if(error != FlyCapture2::PGRERROR_OK)
	{
		ROS_ERROR("Cannot find camera with specified serial number! - %i", serial);
		error = camera.Connect( 0 ); // connect to default cam index 0
	}
	else
	{
		error = camera.Connect(&guid);
	}

	if ( error != FlyCapture2::PGRERROR_OK )
	{
		ROS_ERROR("Failed to connect to camera\n");
		return false;
	}

	// Get the camera info and print it out
	error = camera.GetCameraInfo( &camInfo );
	if ( error != FlyCapture2::PGRERROR_OK )
	{
		ROS_ERROR("Failed to get camera info from camera\n");
		return false;
	}
	std::string tempStr = std::string();
	ROS_INFO_STREAM(camInfo.modelName);
	ROS_INFO_STREAM(camInfo.sensorInfo);
	ROS_INFO("%i", camInfo.serialNumber);

	error = camera.StartCapture();
	if ( error == FlyCapture2::PGRERROR_ISOCH_BANDWIDTH_EXCEEDED )
	{
		ROS_WARN("Bandwidth exceeded\n");
		return false;
	}
	else if ( error != FlyCapture2::PGRERROR_OK )
	{
		ROS_ERROR("Failed to start image capture\n");
		return false;
	}
	else if (error == FlyCapture2::PGRERROR_OK)
	{
		ROS_INFO("Started the image capture");
	}

	ROS_DEBUG("connected camera");
	return true;
}

bool setImageSettings(int x_offset, int y_offset, int width, int height, FlyCapture2::PixelFormat pixelForm, FlyCapture2::Mode mode)
{
	bool retVal = true;

	FlyCapture2::Format7ImageSettings imageSettings;

	//setup the image settings for camera upload
	imageSettings.height = height;
	imageSettings.width = width;
	imageSettings.offsetX = x_offset;
	imageSettings.offsetY = y_offset;
	imageSettings.mode = mode;
	imageSettings.pixelFormat = pixelForm;

	bool isValid;
	FlyCapture2::Format7PacketInfo packetInfo;
	error = camera.ValidateFormat7Settings(&imageSettings, &isValid, &packetInfo);
	if(!isValid)
	{
		ROS_WARN("Format 7 Settings Not Valid For Camera! (continuing)");
	}

	error = camera.SetFormat7Configuration(&imageSettings, packetInfo.recommendedBytesPerPacket);
	if(error == FlyCapture2::PGRERROR_OK)
	{
		retVal = true;
		ROS_INFO("Set the format 7 camera settings specified.");
	}
	else
	{
		retVal = false;
		ROS_ERROR("FAILED TO SET CAMERA'S FORMAT 7 SETTINGS!");
	}

	return retVal;
}

bool setProperty(const FlyCapture2::PropertyType type, const bool autoSet, unsigned int valueA, unsigned int valueB)
{
	// return true if we can set values as desired.
	bool retVal = true;

	FlyCapture2::PropertyInfo pInfo;
	pInfo.type = type;
	error = camera.GetPropertyInfo(&pInfo);
	//handle the error
	//TODO

	if(pInfo.present)
	{
		FlyCapture2::Property prop;
		prop.type = type;
		prop.autoManualMode = (autoSet && pInfo.autoSupported);
		prop.absControl = false;
		prop.onOff = pInfo.onOffSupported;

		if(valueA < pInfo.min)
		{
			valueA = pInfo.min;
			retVal &= false;
		}
		else if(valueA > pInfo.max)
		{
			valueA = pInfo.max;
			retVal &= false;
		}
		if(valueB < pInfo.min)
		{
			valueB = pInfo.min;
			retVal &= false;
		}
		else if(valueB > pInfo.max)
		{
			valueB = pInfo.max;
			retVal &= false;
		}
		prop.valueA = valueA;
		prop.valueB = valueB;
		error = camera.SetProperty(&prop);
		//handle error
		if(error != FlyCapture2::PGRERROR_OK)
		{
			ROS_ERROR("Failed to set parameter");
		}

		// Read back setting to confirm
		error = camera.GetProperty(&prop);
		//handle error
		//TODO

		if(!prop.autoManualMode)
		{
			valueA = prop.valueA;
			valueB = prop.valueB;
		}
	}
	else     // Not supported
	{
		valueA = 0;
		valueB = 0;
	}

	return retVal;
}

bool adjustCameraSettings()
{
	ROS_INFO("Adjusting camera settings for 13S2C camera");
	//set exposure
	setProperty(FlyCapture2::AUTO_EXPOSURE, true, 1, 0);
	//set gain
	setProperty(FlyCapture2::GAIN, true, 1, 0);
	//set shutter
	setProperty(FlyCapture2::SHUTTER, true, 1, 0);
	//set frame rate
	setProperty(FlyCapture2::FRAME_RATE, false, rate, 0);
}

/*
 * This function will take a string containing ; and , and it will create a cv mat from it
 * this creates a CV32FC1 mat by default
 */
cv::Mat createMatFromString(std::string text)
{
	ROS_DEBUG_STREAM("creating MAT from " << text << std::endl);

	//size
	int rows, cols;
	//setup the row strings
	std::vector<std::string> rowStrings;
	std::stringstream textStream(text);
	std::string segment;

	while(std::getline(textStream, segment, ';'))
	{
		rowStrings.push_back(segment);
	}

	rows = (int)rowStrings.size();

	//find the column size by adding for each segment
	std::stringstream rowStream(rowStrings.at(0));
	cols = 0;
	while(std::getline(rowStream, segment, ','))
	{
		cols++;
	}

	ROS_DEBUG_STREAM("the matrix appears to be rows: " << rows << " cols: " << cols);

	//create the cv::mat
	cv::Mat matrix = cv::Mat(rows, cols, CV_32FC1);
	//now cycle through rows and create the cv::Mat
	for(int i = 0; i < rows; i++)
	{
		std::stringstream rowStream(rowStrings.at(i));
		std::string segment;
		for(int j = 0; j < cols; j++)
		{
			std::getline(rowStream, segment, ',');
			//remove whitespace
			segment = removeSpaces(segment);
			//convert and add to matrix
			ROS_DEBUG_STREAM("attempting to convert " << segment << " to a float");
			matrix.at<float>(i, j) = boost::lexical_cast< float >( segment );
			ROS_DEBUG_STREAM("converted to " << matrix.at<float>(i, j));
		}
	}

	ROS_DEBUG_STREAM("created mat. is this correct? " << matrix);

	return matrix;

}

string removeSpaces(string input)
{
	input.erase(std::remove(input.begin(),input.end(),' '),input.end());
	return input;
}
