#include "pose_estim.h"

bool pose_estimator::calc_pose(marker *Marker, pose6D &pose_est)
{
	if(Marker->found)
	{
		pose_est.isvalid = true;
		cv::Mat rvec, tvec;
		solvePnP(object_points, Marker->corner_coord,
			camera_matrix, dist_coeffs, rvec, tvec, false, CV_ITERATIVE);
		    
		pose_est.x = tvec.at<double>(0);
		pose_est.y = tvec.at<double>(2);
		pose_est.z = tvec.at<double>(1);
	}
	else
	{
		pose_est.isvalid = false;
		pose_est.x = 0;
		pose_est.y = 0;
		pose_est.z = 0;
	}
	
	return true;
}

cv::Mat pose_estimator::euler2mat(double roll, double pitch,
	double yaw, std::string order)
{
	cv::Mat res_mat;

	//Calculate rotation about x axis
	cv::Mat rx = (cv::Mat_<double>(3, 3) <<
			  1,	0,			0,
			  0,	cos(roll),	-sin(roll),
			  0,	sin(roll),	 cos(roll));

	//Calculate rotation about y axis
	cv::Mat ry = (cv::Mat_<double>(3, 3) <<
			  cos(pitch),	0,	 sin(pitch),
			  0,			1,	 0,
			  -sin(pitch),	0,	 cos(pitch));

	//Calculate rotation about z axis
	cv::Mat rz = (cv::Mat_<double>(3, 3) <<
			  cos(yaw),	 -sin(yaw),	 0,
			  sin(yaw),	  cos(yaw),	 0,
			  0,	 	  0,	 	 1);

	if (order.compare("ZYX") == 0)
		res_mat = rz * ry * rx;
	else if (order.compare("XYZ") == 0)
		res_mat = rx * ry * rz;

	return res_mat;
}
