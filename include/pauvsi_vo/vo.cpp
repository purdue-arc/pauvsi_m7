/*
 * vo.cpp
 *
 *  Created on: Sep 19, 2016
 *      Author: kevinsheridan
 */

#include "vo.h"

VO::VO(double px, double py, double pz, double qw, double qx, double qy, double qz)
{
	std::vector<double> temp;
	temp.push_back(px);
	temp.push_back(py);
	temp.push_back(pz);
	this->position = temp;

	temp.clear();
	ROS_DEBUG_STREAM("temp size " << temp.size());
	temp.push_back(qw);
	temp.push_back(qx);
	temp.push_back(qy);
	temp.push_back(qz);
	this->orientation = temp;
}

void VO::correctPosition(std::vector<double> pos)
{
	this->position = pos;
}


