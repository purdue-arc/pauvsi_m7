#include <ros/ros.h>
#include <ros/console.h>
#include <flycapture/FlyCapture2.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

int connectCamera();
void getParameters();
int setImageSettings(int x_offset, int y_offset, int width, int height, FlyCapture2::PixelFormat pixelForm, FlyCapture2::Mode mode);
bool setProperty(const FlyCapture2::PropertyType &type, const bool &autoSet, unsigned int &valueA, unsigned int &valueB);
bool adjustCameraSettings(int cameType);

#define DEFAULT_RATE 10
#define CAM_13Y3C 2
#define CAM_13S2C 1

int rate;
int serial;
int camType;

FlyCapture2::Error error;
FlyCapture2::Camera camera;
FlyCapture2::CameraInfo camInfo;
FlyCapture2::BusManager busMngr;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "CameraDriver", ros::init_options::AnonymousName); // initializes with a randomish name
	ros::NodeHandle nh; // create the node handler

	getParameters(); // set all params

	// Connect the camera
	connectCamera();
	//adjust cam settings for best performance
	adjustCameraSettings(camType);

	//set the rate
	ros::Rate loop_rate(rate);
	//the main loop
	while(nh.ok())
	{
		//capture the raw image from cam
		FlyCapture2::Image rawImage;
		error = camera.RetrieveBuffer( &rawImage );
		if ( error != FlyCapture2::PGRERROR_OK )
		{
			ROS_WARN("Capture Error");
			continue;
		}

		// convert to rgb
		FlyCapture2::Image bgrImage;
		rawImage.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &bgrImage );

		// convert to OpenCV Mat
		unsigned int rowBytes = (double)bgrImage.GetReceivedDataSize()/(double)bgrImage.GetRows();
		cv::Mat image = cv::Mat(bgrImage.GetRows(), bgrImage.GetCols(), CV_8UC3, bgrImage.GetData(),rowBytes);

		cv::imshow("image", image);
		cv::waitKey(30);

		//SLEEP
		loop_rate.sleep();
	}

	//disconnect the camera
	camera.Disconnect();

	return 0;
}

void getParameters()
{
	//get the parameters
	//rate
	ros::param::param<int>("frame_rate", rate, DEFAULT_RATE);
	ROS_INFO("Camera Frame Rate: %i", rate);
	//serial
	ros::param::param<int>("serial_number", serial, 0);
	//cam type
	ros::param::param<int>("camera_type", camType, 1);
}

int connectCamera()
{
	FlyCapture2::PGRGuid guid; //id of the camera
	//get the camera with specific serial
	error = busMngr.GetCameraFromSerialNumber(serial, &guid);
	if(error != FlyCapture2::PGRERROR_OK)
	{
		ROS_ERROR("Cannot find camera with specified serial number!");
		error = camera.Connect( 0 ); // connect to default cam index 0
	}
	else
	{
		error = camera.Connect(&guid);
	}

	if ( error != FlyCapture2::PGRERROR_OK )
	{
		ROS_INFO("Failed to connect to camera\n");
		return false;
	}

	// Get the camera info and print it out
	error = camera.GetCameraInfo( &camInfo );
	if ( error != FlyCapture2::PGRERROR_OK )
	{
		ROS_INFO("Failed to get camera info from camera\n");
		return false;
	}
	std::string tempStr = std::string();
	ROS_INFO_STREAM(camInfo.modelName);
	ROS_INFO_STREAM(camInfo.sensorInfo);
	ROS_INFO("%i", camInfo.serialNumber);

	error = camera.StartCapture();
	if ( error == FlyCapture2::PGRERROR_ISOCH_BANDWIDTH_EXCEEDED )
	{
		ROS_INFO("Bandwidth exceeded\n");
		return false;
	}
	else if ( error != FlyCapture2::PGRERROR_OK )
	{
		ROS_INFO("Failed to start image capture\n");
		return false;
	}
	else if (error == FlyCapture2::PGRERROR_OK)
	{
		ROS_INFO("Started the image capture");
	}

	return true;
}

int setImageSettings(int x_offset, int y_offset, int width, int height, FlyCapture2::PixelFormat pixelForm, FlyCapture2::Mode mode)
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
		//TODO

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

bool adjustCameraSettings(int cameType)
{
	if(camType == CAM_13S2C)
	{
		ROS_INFO("Adjusting camera settings for 13S2C camera");
		//set exposure
		setProperty(FlyCapture2::AUTO_EXPOSURE, true, 1, 1);
	}
	else if(camType == CAM_13Y3C)
	{
		ROS_INFO("Adjusting camera settings for 13Y3C camera");
		//set exposure
		setProperty(FlyCapture2::AUTO_EXPOSURE, true, 1, 1);
	}
}
