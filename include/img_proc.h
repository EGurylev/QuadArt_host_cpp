#pragma once

#include "common.h"
#include <opencv2/opencv.hpp>

using namespace std::chrono;

struct marker
{
	friend class img_proc;
	private:
		double area_prev;
		double perimeter_prev = 160;
		int size;
		bool found_prev = false;
	public:
		bool found = false;
		cv::Point coord;
		std::vector<cv::Point2f> corner_coord;
};

struct img_debug
{
	double area;
	double perimeter;
	double min_diff;
};

class img_proc
{
	private:
		const int img_width = 1280;
		const int img_height = 1024;
		const double win_scale = 3;
		double const diff_thresh = 0.2;
		marker Marker;
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
		img_proc();
		marker* marker_search(uint8_t* input_img);
		img_debug log_debug;
};
