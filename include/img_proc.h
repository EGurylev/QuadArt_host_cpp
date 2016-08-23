#pragma once

#include "common.h"
#include <opencv2/opencv.hpp>

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
		cv::Point marker_coord;
		int marker_size;
		cv::Mat img;
		void find_marker();
		void track_marker();
		void find_corners(std::vector<int>& x,
			std::vector<int>& y, cv::Size frame_size);
		void mean_shift(cv::Mat frame);
		void save_img_debug();
		//debug vars
		int cnt = 0;
		int num_debug_img = 50;
		std::vector<cv::Mat> img_array;
        double Duration;
	public:
		bool marker_found = false;
		std::vector<cv::Point2f> corner_coord;	
		img_proc();
		void marker_search(uint8_t* input_img);
};
