#include "common.h"
#include <opencv2/opencv.hpp>

class observer
{
	public:
		observer();
		double update(double measured, double input, bool isvalid);
	private:
		int N;//number of states
		cv::Mat x, u, y, y_m;
		cv::Mat A, B, C, L;
};
