#include "pose_estim.h"

bool pose_estimator::calc_pose(vector<Point2f>& corner_coord,
	Mat& rvec, Mat& tvec)
{
	solvePnP(object_points, corner_coord,
        camera_matrix, dist_coeffs, rvec, tvec, false, CV_P3P);
    
    return true;
}

Mat pose_estimator::euler2mat(double roll, double pitch,
	double yaw, string order)
{
	Mat res_mat;

	//Calculate rotation about x axis
	Mat rx = (Mat_<double>(3, 3) <<
			  1,	0,			0,
			  0,	cos(roll),	-sin(roll),
			  0,	sin(roll),	 cos(roll));

	//Calculate rotation about y axis
	Mat ry = (Mat_<double>(3, 3) <<
			  cos(pitch),	0,	 sin(pitch),
			  0,			1,	 0,
			  -sin(pitch),	0,	 cos(pitch));

	//Calculate rotation about z axis
	Mat rz = (Mat_<double>(3, 3) <<
			  cos(yaw),	 -sin(yaw),	 0,
			  sin(yaw),	  cos(yaw),	 0,
			  0,	 	  0,	 	 1);

	if (order.compare("ZYX") == 0)
		res_mat = rz * ry * rx;
	else if (order.compare("XYZ") == 0)
		res_mat = rx * ry * rz;

	return res_mat;
}
