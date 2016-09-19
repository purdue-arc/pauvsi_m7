/*
 * vo.cpp
 *
 *  Created on: Sep 19, 2016
 *      Author: kevinsheridan
 */

#include "vo.h"

VO::VO()
{

}

void VO::correctPosition(std::vector<double> pos)
{
	this->position = pos;
}

void VO::passNodeHandle(ros::NodeHandle nh)
{
	this->nh = nh;
}


