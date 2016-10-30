/*
Observer based on Kalman filter with constant
optimal gain precalculated offline based on
system model and model and noise variances.
It initialized from text file.
*/

#include "observer.h"

observer::observer()
{
	
	std::ifstream reader("observer");
	reader >> N;//number of states
	
	x = cv::Mat::zeros(N, 1, CV_64F);
	y = cv::Mat::zeros(1, 1, CV_64F);
	y_m = cv::Mat::zeros(1, 1, CV_64F);
	u = cv::Mat::zeros(1, 1, CV_64F);
	
	A = cv::Mat::zeros(N, N, CV_64F);
	B = cv::Mat::zeros(N, 1, CV_64F);
	C = cv::Mat::zeros(1, N, CV_64F);
	L = cv::Mat::zeros(N, 1, CV_64F);
	
	//fill A matrix
	double read_value;
	for (int i = 0; i < N; i++)
		for (int j = 0; j < N; j++)
		{
			reader >> read_value;
			A.at<double>(i, j) = read_value;
		}
	//fill B matrix
	for (int i = 0; i < N; i++)
	{
		reader >> read_value;
		B.at<double>(i, 0) = read_value;
	}
	//fill C matrix
	for (int i = 0; i < N; i++)
	{
		reader >> read_value;
		C.at<double>(0, i) = read_value;
	}
	//fill L matrix
	for (int i = 0; i < N; i++)
	{
		reader >> read_value;
		L.at<double>(i, 0) = read_value;
	}
	reader.close();
}

double observer::update(double measured, double input,  bool isvalid)
{
	if (isvalid)
	{
		y_m.at<double>(0, 0) = measured;
		u.at<double>(0, 0) = input;
		x = A * x + L * (y_m - C * x) + B * u;
		y = C * x;
	}
	
	return y.at<double>(0, 0);
}
