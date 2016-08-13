/*
Class for solving Perspective-n-Point problem (PnP).
It uses solvePnP function from OpenCV library.
*/

#pragma once

#include "common.h"

class pose_estimator
{
	private:
		double l = 3.0;//length of a marker in cm
		vector<Point3f> object_points = 
			{ Point3f(-l/2, -l/2, 0),
			  Point3f( l/2, -l/2, 0),
			  Point3f(-l/2,  l/2, 0),
			  Point3f( l/2,  l/2, 0) };
			  
		double _cm[9] = { 837.62249756, 0,  581.72735113,
                      	  0,  834.21710205, 536.5862742,
                      	  0,  0,  1 };
        Mat camera_matrix = Mat(3, 3, CV_64FC1, _cm);
        
        
        double _dc[5] = { -5.03037690e-01,
        				   4.90417149e-01,
        				   1.09398451e-02,
        				  -3.89431007e-05,
        				  -2.62931231e-01 };
        Mat dist_coeffs = Mat(5, 1, CV_64FC1, _dc);
      
    public:
    	bool calc_pose(vector<Point2f>& corner_coord,
    		Mat& rvec, Mat& tvec);
    		
    	static Mat euler2mat(double roll, double pitch,
    		double yaw, string order);
};
