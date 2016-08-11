#include "pose_estim.h"

bool pose_estimator::calc_pose(vector<Point2f>& corner_coord, Mat& rvec, Mat& tvec)
{
	solvePnP(object_points, corner_coord,
        camera_matrix, dist_coeffs, rvec, tvec, false, CV_P3P);
    
    return true;
}
