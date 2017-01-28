
#ifndef PAUVSI_TRACKER_INCLUDE_KALMAN_FILTER_H_
#define PAUVSI_TRACKER_INCLUDE_KALMAN_FILTER_H_

#include <opencv2/calib3d.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/video.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <eigen3/Eigen/Geometry>
#include <stdio.h>
#include <boost/lexical_cast.hpp>
#include <sensor_msgs/Image.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <vector>
#include <string.h>

/*
 * If you don't get new measurements, do the prediction step but only change the error covariance once:
 * 1. x = fx(t-1)
 * 	  P = FPF(transpose) + Q
 * From 2. onwards:  x = Fx(t-1)
 */


class KalmanFilter
{
  public:
	KalmanFilter(double dt,
				 const Eigen::MatrixXd& F,
				 const Eigen::MatrixXd& H,
				 const Eigen::MatrixXd& Q,
				 const Eigen::MatrixXd& R,
				 const Eigen::MatrixXd& P);

	KalmanFilter();

	void init(double t0, const Eigen::VectorXd& x0);

	void predict();
	void update(const Eigen::VectorXd& y);

 private:
	 /**
	  * Create a Kalman filter with the specified matrices.
	  *   F - System dynamics matrix
	  *   H - Output matrix (Gaussian measurement combination matrix)
	  *   Q - Process noise covariance
	  *   R - Measurement noise covariance
	  *   P - Estimated error covariance
	  *   K - Kalman Gain
	  */
	Eigen::MatrixXd F, H, Q, R, P, K, P0;

	//Dimensions
	int m, n;

	//n-size Identity
	Eigen::MatrixXd I;

	//Initial and current time
	double t0, t;

	//Time step
	double dt;

	//Is filter initialized?
	bool initialized;

	//Estimated States
	Eigen::VectorXd x_hat, x_hat_new;
};
