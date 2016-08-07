#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <chrono>
#include <fstream>
#include <iostream>
#include <QtCore/QObject>

using namespace cv;
using namespace std;
using namespace std::chrono;

class img_proc
{
	private:
		const int img_width = 1280;
		const int img_height = 1024;
		int cnt = 0;
		int num_debug_img = 50;
		Mat img;
		vector<Mat> img_array;
        double Duration;
	public:
		vector<int> marker_coord;
		bool marker_found = false;
		void marker_search(uint8_t* input_img);
		void find_marker();
		void track_marker();
		void find_corners();
		void mean_shift();
		void save_img_debug();
};
