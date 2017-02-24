/*
 * Created On: 27 January 2017
 * Author: Logesh Roshan Ramadoss
 */
#include "kalman_filter.hpp"



KalmanFilter::KalmanFilter(double dt,
						   const Eigen::MatrixXd& F,
						   const Eigen::MatrixXd& H,
						   const Eigen::MatrixXd& Q,
						   const Eigen::MatrixXd& R,
						   const Eigen::MatrixXd& P)
{
	this->F(F);
	this->H(H);
	this->Q(Q);
	this->R(R);
	this->P0(P);
	this->dt = dt;
	this->m = H.rows();
	this->n = F.rows();
	initialized = false;
	I(n,n);
	I.setIdentity();
	x_hat(n); x_hat_new(n);
}

void KalmanFilter::init(double t0, const Eigen::VectorXd& x0)
{
	x_hat = x0;
	P = P0;
	this->t0 = t0;
	t = t0;
	initialized = true;

}

void KalmanFilter::predict()
{
	if(!initialized)
	{
		ROS_DEBUG_STREAM("NOT INITIALIZED");
		return;
	}

	x_hat_new = F * x_hat;
	P = F*P*F.transpose() + Q;

}

void KalmanFilter::update(const Eigen::MatrixXd& y)
{
	if(!initialized)
	{
		ROS_DEBUG_STREAM("NOT INITIALIZED");
		return;
	}
	//predict
//	x_hat_new = F * x_hat;
//	P = F*P*F.transpose() + Q;

	Eigen::MatrixXd S = H*P*H.transponse() + R;
	K = P*H.transpose()*S.inverse();
	x_hat_new += K*(y - H*x_hat_new);
	P = (I - K*H)*P;
	x_hat = x_hat_new;


	t = t+dt;
}

geometry_msgs::PoseStamped KalmanFilter::getPoseStamped()
{
	geometry_msgs::PoseStamped pose;// = new geometry_msgs::PoseStamped(x_hat[0], x_hat[1], 0);
	pose.header.stamp = ros::Time(ros::Time(0));
	pose.pose.position.x = x_hat(0, 0);
	pose.pose.position.y = x_hat(1, 0);
	pose.pose.position.z = ROOMBA_HEIGHT;

	return pose;
}

geometry_msgs::PoseWithCovarianceStamped KalmanFilter::getPoseWithCovariance()
{
	geometry_msgs::PoseWithCovarianceStamped pose;
	pose.header.stamp =ros::Time(ros::Time(0));
	pose.pose.pose.position.x = x_hat(0, 0);
	pose.pose.pose.position.y = x_hat(1, 0);
	pose.pose.pose.position.z = ROOMBA_HEIGHT;
	pose.pose.covariance = P;

	return pose;

}
