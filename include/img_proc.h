#pragma once

#include "common.h"

using namespace std::chrono;

class img_proc
{
	private:
		const int img_width = 1280;
		const int img_height = 1024;
		double area_prev;
		double perimeter_prev = 160;
		const double win_scale = 3;
		double const diff_thresh = 0.2;
		bool marker_found_prev = false;
		Point marker_coord;
		int marker_size;
		Mat img;
		void find_marker();
		void track_marker();
		void find_corners(vector<int>& x, vector<int>& y, Size frame_size);
		void mean_shift(Mat frame);
		void save_img_debug();
		//debug vars
		int cnt = 0;
		int num_debug_img = 50;
		vector<Mat> img_array;
        double Duration;
	public:
		bool marker_found = false;
		vector<Point2f> corner_coord;	
		img_proc();
		void marker_search(uint8_t* input_img);
};
