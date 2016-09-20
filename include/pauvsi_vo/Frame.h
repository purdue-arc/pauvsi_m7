/*
 * frame.h
 *
 *  Created on: Sep 20, 2016
 *      Author: kevinsheridan
 */

#ifndef PAUVSI_M7_INCLUDE_PAUVSI_VO_FRAME_H_
#define PAUVSI_M7_INCLUDE_PAUVSI_VO_FRAME_H_

#include <vo.h>

class Frame
{

private:

public:
	ros::Time timeCreated;
	cv::Mat image;
	std::vector<cv::Point2d> corners;

	Frame(cv::Mat img, ros::Time t)
	{
		this->image = img;
		this->timeCreated = t;
	}
	/*
	 * gets the time since this frame was create
	 */
	double getAgeSeconds()
	{
		return ros::Time::now().toSec() - this->timeCreated.toSec();
	}

};



#endif /* PAUVSI_M7_INCLUDE_PAUVSI_VO_FRAME_H_ */
