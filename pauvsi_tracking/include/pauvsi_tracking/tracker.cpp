/*
 * tracker.cpp
 *
 *  Created On: Nov 9, 2016
 *  	Author: Logesh Roshan Ramadoss
 *
 */

#include "tracker.h"

Tracker::Tracker()
{
	image_transport::ImageTransport it(nh);
	this->cameraSub = it.subscribeCamera(this->getCameraTopic(), 1, &Tracker::cameraCallback, this);

}

void Tracker::cameraCallback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& cam)
{

	this->setK(get3x3FromVector(cam->K));
	this->inputImg = cv_bridge::toCvShare(img, "mono8")->image.clone();

}
