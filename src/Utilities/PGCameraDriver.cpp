#include <ros.h>
#include <flycapture/FlyCapture2.h>

FlyCapture2::Camera cam;
FlyCapture2::CameraInfo cInfo;

int main()
{
	cam.GetCameraInfo(&cInfo);

	return 0;
}


