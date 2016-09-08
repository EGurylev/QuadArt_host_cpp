#include "pose_estim.h"
#include <math.h>


void pose_estimator::calc_pose(marker *Marker, pose6D &pose_est)
{
	if(Marker->found)
	{
		//Estimate position
		pose_est.isvalid = true;
		cv::Mat rvec, tvec;
		solvePnP(object_points, Marker->corner_coord,
			camera_matrix, dist_coeffs, rvec, tvec, false, CV_ITERATIVE);

		//Calc. the difference between model and measurements
		cv::Mat proj_points;
		cv::projectPoints(object_points, rvec, tvec,
			camera_matrix, dist_coeffs, proj_points);

		double proj_error = 0;
		for(int i = 0; i < 4; i++)
		{
			cv::Point2f vec_diff = proj_points.at<cv::Point2f>(i) -
				Marker->corner_coord.at(i);
			proj_error += sqrt(vec_diff.x * vec_diff.x +
				vec_diff.y * vec_diff.y);
        }

		if(proj_error > proj_err_thresh)
			pose_est.isvalid = false;

		pose_est.x = tvec.at<double>(0);
		pose_est.y = tvec.at<double>(2);
		pose_est.z = -tvec.at<double>(1);

		//Log debug variables
		log_debug.proj_error = proj_error;
	}
	else
	{
		pose_est.isvalid = false;
		pose_est.x = 0;
		pose_est.y = 0;
		pose_est.z = 0;
	}
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
