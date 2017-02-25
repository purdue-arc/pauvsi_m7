#include "controller.hpp"


Controller::Controller()
{
	redObjectTrackers[0] = *(new Tracker(CAMERA_TOPIC_1, CAMERA_FRAME_1, REDILOWHUE, REDIHIGHHUE,
								REDILOWSATURATION, REDIHIGHSATURATION, REDILOWVALUE, REDIHIGHVALUE));
	redObjectTrackers[1] = *(new Tracker(CAMERA_TOPIC_2, CAMERA_FRAME_2, REDILOWHUE, REDIHIGHHUE,
								REDILOWSATURATION, REDIHIGHSATURATION, REDILOWVALUE, REDIHIGHVALUE));
	redObjectTrackers[2] = *(new Tracker(CAMERA_TOPIC_3, CAMERA_FRAME_3, REDILOWHUE, REDIHIGHHUE,
								REDILOWSATURATION, REDIHIGHSATURATION, REDILOWVALUE, REDIHIGHVALUE));
	redObjectTrackers[3] = *(new Tracker(CAMERA_TOPIC_4, CAMERA_FRAME_4, REDILOWHUE, REDIHIGHHUE,
								REDILOWSATURATION, REDIHIGHSATURATION, REDILOWVALUE, REDIHIGHVALUE));
	redObjectTrackers[4] = *(new Tracker(CAMERA_TOPIC_5, CAMERA_FRAME_5, REDILOWHUE, REDIHIGHHUE,
								REDILOWSATURATION, REDIHIGHSATURATION, REDILOWVALUE, REDIHIGHVALUE));

	greenObjectTrackers[0] = *(new Tracker(CAMERA_TOPIC_1, CAMERA_FRAME_1, GREENILOWHUE, GREENIHIGHHUE,
										GREENILOWSATURATION, GREENIHIGHSATURATION, GREENILOWVALUE, GREENIHIGHVALUE));
	greenObjectTrackers[1] = *(new Tracker(CAMERA_TOPIC_2, CAMERA_FRAME_2, GREENILOWHUE, GREENIHIGHHUE,
										GREENILOWSATURATION, GREENIHIGHSATURATION, GREENILOWVALUE, GREENIHIGHVALUE));
	greenObjectTrackers[2] = *(new Tracker(CAMERA_TOPIC_3, CAMERA_FRAME_3, GREENILOWHUE, GREENIHIGHHUE,
										GREENILOWSATURATION, GREENIHIGHSATURATION, GREENILOWVALUE, GREENIHIGHVALUE));
	greenObjectTrackers[3] = *(new Tracker(CAMERA_TOPIC_4, CAMERA_FRAME_4, GREENILOWHUE, GREENIHIGHHUE,
										GREENILOWSATURATION, GREENIHIGHSATURATION, GREENILOWVALUE, GREENIHIGHVALUE));
	greenObjectTrackers[4] = *(new Tracker(CAMERA_TOPIC_5, CAMERA_FRAME_5, GREENILOWHUE, GREENIHIGHHUE,
										GREENILOWSATURATION, GREENIHIGHSATURATION, GREENILOWVALUE, GREENIHIGHVALUE));

	// INITIALIZE KALMAN FILTERS



	this->imageHeader = redObjectTrackers[0].getHeader();

	this->init();
}

//returns true if the first argument goes before the second argument in the strict weak ordering
static bool wayToSort(tf::Vector3 i, tf::Vector3 j)
{
	//bool v = i.quality<j.quality;
	return (i.getX()<j.getX());
}

void Controller::removeCopies()
{
	//
	for(int i=0; i<5; ++i)
	{
		for(auto& e: redObjectTrackers[i].getPoses())
		{
			uniqueRedPoses.push_front(e);
		}
		for(auto& e: greenObjectTrackers[i].getPoses())
		{
			uniqueGreenPoses.push_front(e);
		}

	}

	uniqueRedPoses.sort(wayToSort);
	uniqueGreenPoses.sort(wayToSort);

	std::list<tf::Vector3>::iterator it = uniqueRedPoses.begin();
	std::list<tf::Vector3>::iterator nextIt = uniqueRedPoses.begin()++;
	std::list<tf::Vector3>::iterator nextToNext = std::next(it, 2);


	while(nextIt != uniqueRedPoses.end())
	{

		if(!(((*nextIt).getX()-(*it).getX() < SPACE_BETWEEN_ROOMBA) &&
				((*nextIt).getY() - (*it).getY() < SPACE_BETWEEN_ROOMBA)))
		{

		}
		//If the x and y distance b/w the two roombas is smaller than the threshold
		else
		{
			if(nextToNext != uniqueRedPoses.end())
			{
				if(!(((*nextToNext).getX() - (*nextIt).getX() < SPACE_BETWEEN_ROOMBA) &&
									((*nextToNext).getY() - (*nextIt).getY() < SPACE_BETWEEN_ROOMBA)))
				{
					(*it).setX(((*it).getX() + (*nextIt).getX()) / 2);
					(*it).setY(((*it).getY() + (*nextIt).getY()) / 2);
					uniqueRedPoses.erase(nextIt);

				}

			// All three points are either same or they seem to be chained together, only consider a single chain (3 elts)
				else
				{
					(*it).setX(((*it).getX() + (*nextIt).getX() + (*nextToNext).getX()) / 3);
					(*it).setY(((*it).getY() + (*nextIt).getY() + (*nextToNext).getY()) / 3);
					uniqueRedPoses.erase(nextIt);
					uniqueRedPoses.erase(nextToNext);
				}

			}
			else
			{
				(*it).setX(((*it).getX() + (*nextIt).getX()) / 2);
				(*it).setY(((*it).getY() + (*nextIt).getY()) / 2);
				uniqueRedPoses.erase(nextIt);
			}

		}

		++it;
		nextIt = std::next(it, 1);
		nextToNext = std::next(it, 2);
	}


	it = uniqueGreenPoses.begin();
	nextIt = uniqueGreenPoses.begin()++;
	nextToNext = std::next(it, 2);

	while(nextIt != uniqueGreenPoses.end())
	{
		if(!(((*nextIt).getX()-(*it).getX() < SPACE_BETWEEN_ROOMBA) &&
				((*nextIt).getY() - (*it).getY() < SPACE_BETWEEN_ROOMBA)))
		{

		}
		//If the x and y distance b/w the two roombas is smaller than the threshold
		else
		{
			if(nextToNext != uniqueGreenPoses.end())
			{
				if(!(((*nextToNext).getX() - (*nextIt).getX() < SPACE_BETWEEN_ROOMBA) &&
									((*nextToNext).getY() - (*nextIt).getY() < SPACE_BETWEEN_ROOMBA)))
				{
					(*it).setX(((*it).getX() + (*nextIt).getX()) / 2);
					(*it).setY(((*it).getY() + (*nextIt).getY()) / 2);
					uniqueGreenPoses.erase(nextIt);

				}

			// All three points are either same or they seem to be chained together, only consider a single chain (3 elts)
				else
				{
					(*it).setX(((*it).getX() + (*nextIt).getX() + (*nextToNext).getX()) / 3);
					(*it).setY(((*it).getY() + (*nextIt).getY() + (*nextToNext).getY()) / 3);
					uniqueGreenPoses.erase(nextIt);
					uniqueGreenPoses.erase(nextToNext);
				}

			}
			else
			{
				(*it).setX(((*it).getX() + (*nextIt).getX()) / 2);
				(*it).setY(((*it).getY() + (*nextIt).getY()) / 2);
				uniqueGreenPoses.erase(nextIt);
			}

		}

		++it;
		nextIt = std::next(it, 1);
		nextToNext = std::next(it, 2);
	}


	//Go through the list, remove copies and average copy positions. Makes each roomba position unique.
//	for(std::list<tf::Vector3>::iterator it=uniqueRedPoses.begin(); it+1 != uniqueRedPoses.end(); ++it )
//	{
		/*//If the x and y distance b/w the two roombas is smaller than the threshold
		if(!(((*(it+1)).getX()-(*it).getX() < SPACE_BETWEEN_ROOMBA) &&
				((*(it+1)).getY() - (*it).getY() < SPACE_BETWEEN_ROOMBA)))
		{

		}
		else
		{
			// They are the same

			if((*(it+2)) != uniqueRedPoses.end())
			{

				if(!(((*(it+2)).getX() - (*(it+1)).getX() < SPACE_BETWEEN_ROOMBA) &&
									((*(it+2)).getY() - (*(it+1)).getY() < SPACE_BETWEEN_ROOMBA)))
				{
					(*it).setX(((*it).getX() + (*(it+1)).getX()) / 2);
					(*it).setY(((*it).getY() + (*(it+1)).getY()) / 2);
					uniqueRedPoses.erase(it+1);

				}
				// All three points are either same or they seem to be chained together, only consider a single chain (3 elts)
				else
				{
					(*it).setX(((*it).getX() + (*(it+1)).getX() + (*(it+2)).getX()) / 3);
					(*it).setY(((*it).getY() + (*(it+1)).getY() + (*(it+2)).getY()) / 3);
					uniqueRedPoses.erase(it+1);
					uniqueRedPoses.erase(it+2);
				}
			}
			else
			{
				(*it).setX(((*it).getX() + (*(it+1)).getX()) / 2);
				(*it).setY(((*it).getY() + (*(it+1)).getY()) / 2);
				uniqueRedPoses.erase(it+1);
			}
		}*/
//	}

/*
	//Go through the list, remove copies and average copy positions. Makes each roomba position unique.
	for(std::list<tf::Vector3>::iterator it=uniqueGreenPoses.begin(); it+1 != uniqueGreenPoses.end(); ++it )
	{
		//If the x and y distance b/w the two roombas is smaller than the threshold
		if(!(((*(it+1)).getX()-(*it).getX() < SPACE_BETWEEN_ROOMBA) &&
				((*(it+1)).getY() - (*it).getY() < SPACE_BETWEEN_ROOMBA)))
		{

		}
		else
		{
			// They are the same

			if((*(it+2)) != uniqueGreenPoses.end())
			{

				if(!(((*(it+2)).getX() - (*(it+1)).getX() < SPACE_BETWEEN_ROOMBA) &&
									((*(it+2)).getY() - (*(it+1)).getY() < SPACE_BETWEEN_ROOMBA)))
				{
					(*it).setX(((*it).getX() + (*(it+1)).getX()) / 2);
					(*it).setY(((*it).getY() + (*(it+1)).getY()) / 2);
					uniqueGreenPoses.erase(it+1);

				}
				// All three points are either same or they seem to be chained together, only consider a single chain (3 elts)
				else
				{
					(*it).setX(((*it).getX() + (*(it+1)).getX() + (*(it+2)).getX()) / 3);
					(*it).setY(((*it).getY() + (*(it+1)).getY() + (*(it+2)).getY()) / 3);
					uniqueGreenPoses.erase(it+1);
					uniqueGreenPoses.erase(it+2);
				}
			}
			else
			{
				(*it).setX(((*it).getX() + (*(it+1)).getX()) / 2);
				(*it).setY(((*it).getY() + (*(it+1)).getY()) / 2);
				uniqueGreenPoses.erase(it+1);
			}
		}
	}

*/
}

void Controller::updateKF()
{

	//Go through unique poses, find matching positions and update the Kalman Filters
	//What if I don't get a measurement for a roomba? ANS: Find out through empty list
	kFilter[0].getPoseStamped();
	float min = FLT_MAX;
	std::list<tf::Vector3>::iterator pos;
	float displacement;
	geometry_msgs::PoseStamped currPose;

	for(int i=0; i<5; ++i)
	{
		if(uniqueRedPoses.empty())
			break;

		currPose = kFilter[i].getPoseStamped();
		std::list<tf::Vector3>::iterator it = uniqueRedPoses.begin();

		while(it != uniqueRedPoses.end())
		{
			displacement = (currPose.pose.position.x - (*it).getX())*(currPose.pose.position.x - (*it).getX()) +
					   (currPose.pose.position.x - (*it).getY())*(currPose.pose.position.x - (*it).getY());
			if(displacement < min)
			{
				min = displacement;
				pos = it;
			}

				++it;
		}
		kFilter[i].update(Eigen::Matrix<double, 2, 1>((*pos).getX(), (*pos).getY()), imageHeader);
		uniqueRedPoses.erase(pos);
	}

	min = FLT_MAX;

	for(int i=5; i<10; ++i)
	{
		if(uniqueGreenPoses.empty())
			break;

		currPose = kFilter[i].getPoseStamped();
		std::list<tf::Vector3>::iterator it = uniqueGreenPoses.begin();

		while(it != uniqueGreenPoses.end())
		{
			displacement = (currPose.pose.position.x - (*it).getX())*(currPose.pose.position.x - (*it).getX()) +
						   (currPose.pose.position.x - (*it).getY())*(currPose.pose.position.x - (*it).getY());
			if(displacement < min)
			{
				min = displacement;
				pos = it;
			}
			++it;
		}

		kFilter[i].update(Eigen::Matrix<double, 2, 1>((*pos).getX(), (*pos).getY()), imageHeader);
		uniqueGreenPoses.erase(pos);
	}

}
