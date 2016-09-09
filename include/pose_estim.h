/*
Class for solving Perspective-n-Point problem (PnP).
It uses solvePnP function from OpenCV library.
*/

#pragma once

#include "common.h"
#include "img_proc.h"

struct pose6D
{
	double x = 0;
	double y = 0;
	double z = 0;
	double roll = 0;
	double pitch = 0;
	double yaw = 0;
	bool isvalid = true;
	double time_stamp = 0;
};

struct pose_debug
{
	double proj_error;
};

class pose_estimator
{
	private:
		double l = 3.0;//length of a marker in cm
		double proj_err_thresh = 6;
		std::vector<cv::Point3f> object_points = 
			{ cv::Point3f(-l/2, -l/2, 0),
			  cv::Point3f( l/2, -l/2, 0),
			  cv::Point3f(-l/2,  l/2, 0),
			  cv::Point3f( l/2,  l/2, 0) };
			  
		double _cm[9] = { 837.62249756, 0,  581.72735113,
                      	  0,  834.21710205, 536.5862742,
                      	  0,  0,  1 };
        cv::Mat camera_matrix = cv::Mat(3, 3, CV_64FC1, _cm);
        
        
        double _dc[5] = { -5.03037690e-01,
        				   4.90417149e-01,
        				   1.09398451e-02,
        				  -3.89431007e-05,
        				  -2.62931231e-01 };
        cv::Mat dist_coeffs = cv::Mat(5, 1, CV_64FC1, _dc);
      
    public:
    	pose_debug log_debug;
    	void calc_pose(marker *Marker, pose6D &pose_est);
    		
    	static cv::Mat euler2mat(double roll, double pitch,
    		double yaw, std::string order);
};
