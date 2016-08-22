#include "img_proc.h"

img_proc::img_proc()
{
	marker_size = perimeter_prev / 4;
	area_prev = marker_size * marker_size;
	marker_coord.x = img_width / 2;
	marker_coord.y = img_height / 2;
	corner_coord.resize(4);
}

void img_proc::marker_search(uint8_t* input_img)
{
	//Convert to OpenCV format
	
	img = cv::Mat(img_height, img_width, CV_8UC3, input_img);
	
	if (marker_found)
		track_marker();
	else
		find_marker();
	
	auto now = std::chrono::system_clock::now();
	auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
	auto epoch = now_ms.time_since_epoch();
	auto value = std::chrono::duration_cast<std::chrono::milliseconds>(epoch);
	long duration = value.count();
	putText(img, std::to_string(duration), cv::Point2f(100, 500),
		cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(255,255,255), 2);
	if (marker_found)
	{
		img_array.push_back(img.clone());
		cnt += 1;
	}
	if (cnt == num_debug_img)//very bad
		save_img_debug();
	
	
	marker_found_prev = marker_found;
	//cout << marker_found << endl;
}

void img_proc::find_marker()
{
	mean_shift(img);
	if (marker_found)
	{
		for (int i = 0; i < 4; i++)
			circle(img, corner_coord[i], 3, cv::Scalar(0, 255, 0), -1);
	}
}

void img_proc::track_marker()
{
	marker_size = perimeter_prev / 4;
	//Set ROI near found marker from previous iteration
	int x = marker_coord.x - static_cast<int>(win_scale * marker_size / 2);
	int y = marker_coord.y - static_cast<int>(win_scale * marker_size / 2);   
	if (y < 0)
		y = 0;
	if (x < 0)
    	x = 0;
    
	int size = static_cast<int>(win_scale * marker_size);
	cv::Rect roi(x, y, size, size);
	cv::Mat marker_frame = img(roi);
    
	mean_shift(marker_frame);
	//Back to global coordinates
	marker_coord.x += x;
	marker_coord.y += y;
	for (int i = 0; i < 4; i++)
	{
		corner_coord[i].x += x;
		corner_coord[i].y += y;
	}
	
	if (marker_found)
	{
		rectangle(img, roi, cv::Scalar(255), 1, 8, 0);
		for (int i = 0; i < 4; i++)
			circle(img, corner_coord[i], 3, cv::Scalar(0, 255, 0), -1);
	}
}

void img_proc::mean_shift(cv::Mat frame)
{
	// Create a new matrix to hold the gray image
	high_resolution_clock::time_point t1 = high_resolution_clock::now();
	cv::Mat gray;
	cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
	cv::Size frame_size = gray.size();
	cv::Mat blurred;
	blur(gray, blurred, cv::Size(10, 10));
	cv::Mat thresholded;
	cv::adaptiveThreshold(blurred, thresholded, 255,
		cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 27,-2);
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(thresholded, contours, cv::RETR_EXTERNAL,
		cv::CHAIN_APPROX_NONE);
	
	if (contours.empty())
		return;
	
	std::vector<double> area, perimeter;
	for (auto cont = contours.begin(); cont != contours.end(); cont++)
	{
		area.push_back(cv::contourArea(*cont));
		perimeter.push_back(cv::arcLength(*cont, true));
	}
	// The etalon marker's area and perimeter comes from previous found marker
	std::vector<double> diff(contours.size());
	for (int i = 0; i < contours.size(); i++)
	{
		double area_diff = abs(area[i] - area_prev) / area_prev;
		double perimeter_diff = abs(perimeter[i] - perimeter_prev) / perimeter_prev;
		//Such simple mixture gives acceptable result
		diff[i] = area_diff + perimeter_diff;
	}
	
	auto min_idx = min_element(diff.begin(), diff.end());
	if (*min_idx > diff_thresh)
	{
		marker_found = false;
		return;
	}
	else
		marker_found = true;
    
	auto idx = min_idx - diff.begin();
	area_prev = area[idx];
	perimeter_prev = perimeter[idx];
    
	//Contour of interest
	auto cont_min = contours[idx];
    
	//Individual vectors for holding x and y coordinates #todo: make a function
	std::vector<int> x_coord(cont_min.size()), y_coord(cont_min.size());
	for (int i = 0; i < cont_min.size(); i++)
	{
		x_coord[i] = cont_min[i].x;
		y_coord[i] = cont_min[i].y;
	}
	//Coordinates of a marker's center
	marker_coord.x = accumulate(x_coord.begin(), x_coord.end(), 0) / x_coord.size();
	marker_coord.y = accumulate(y_coord.begin(), y_coord.end(), 0) / y_coord.size();
    
	/* An alternative approach for calculation center of a contour
	cv::Moments M = cv::moments(cont_min);
	marker_coord.x = M.m10 / M.m00;
	marker_coord.y = M.m01 / M.m00;
	*/
	
	//Can be optimized: choose roi around contour of interest 
	find_corners(x_coord, y_coord, frame_size);
	
	high_resolution_clock::time_point t2 = high_resolution_clock::now();
	Duration = duration_cast<microseconds>(t2 - t1).count();
	std::cout << Duration << std::endl;
}

void img_proc::find_corners(std::vector<int>& x, std::vector<int>& y, cv::Size frame_size)
{
	/*Make Mat object (image) from contour: fill zeros by default
	and ones at contours' points*/
	cv::Mat cont_img;
	cont_img = cv::Mat::zeros(frame_size, CV_32FC1);
	for (int i = 0; i < x.size(); i++)
		cont_img.at<float>(y[i], x[i]) = 1.0;
	//Apply Harris corner detector
	cv::Mat corn_img, bin_img;
	cv::cornerHarris(cont_img, corn_img, 15, 15, 0.1);
	//and make binary image by thresholding
	double min, max;
	cv::minMaxLoc(corn_img, &min, &max);
	cv::threshold(corn_img, bin_img, 0.4 * max, 1, 0);
	//find contours and sort them by area, choose 4 biggest
	bin_img.convertTo(bin_img, CV_8UC1);
	std::vector<std::vector<cv::Point>> contours_corn;
	cv::findContours(bin_img, contours_corn, cv::RETR_EXTERNAL,
		cv::CHAIN_APPROX_NONE);
	
	if (contours_corn.size() < 4)
	{
		marker_found = false;
		return;
	}
	else
	{
		marker_found = true;
    	/* Nested vector of cont_param has 3 ints: contour area, x and y
    	coordinate of contour's center. Sort them by area.
    	*/
    	std::vector<std::vector<int>> cont_param;
    	
    	//Iterate through each found contour
    	for (int cont_cnt = 0; cont_cnt < contours_corn.size(); cont_cnt++)
    	{
    		std::vector<int> x_coord, y_coord;
    		for (int i = 0; i < contours_corn[cont_cnt].size(); i++)
    		{
    			x_coord.push_back(contours_corn[cont_cnt][i].x);
    			y_coord.push_back(contours_corn[cont_cnt][i].y);
    		}
    		//Coordinates of a marker's center
    		int x = accumulate(x_coord.begin(), x_coord.end(), 0) / x_coord.size();
    		int y = accumulate(y_coord.begin(), y_coord.end(), 0) / y_coord.size();
    			
    		std::vector<int> temp_vec;
    		int area = static_cast<int>(contourArea(contours_corn[cont_cnt]));
    		temp_vec.push_back(area);
    		temp_vec.push_back(x);
    		temp_vec.push_back(y);
    		
    		cont_param.push_back(temp_vec);
    	} 
    		
    	//Sort final cont_param vector by area (0-th element)
    	sort(cont_param.begin(), cont_param.end(),
           	[](const std::vector<int>& a, const std::vector<int>& b)
            {
  				return a[0] > b[0];
			});
			
		//Remove N - 4 contours with smallest area
		if (cont_param.size() > 4)
			for (int i = 0; i < (cont_param.size() - 4); i++)
				cont_param.pop_back();
		
		/*cout << "Sorted vector" << endl;
		for (int i = 0; i < cont_param.size();i++)
		{
			int a = cont_param[i][0];
			int b = cont_param[i][1];
			int c = cont_param[i][2];
			std::cout << a << " " << b << " " << c << std::endl;
		}*/
		
		//Vector for holding corner coordinates
		std::vector<cv::Point> corner_coord_temp;
		for (int i = 0; i < 4; i++)
		{
			cv::Point point;
			point.x = cont_param[i][1];//x
			point.y = cont_param[i][2];//y
			corner_coord_temp.push_back(point);
		}
		
		/*cout << "Temporal vector" << endl;
		for (int i = 0; i < corner_coord_temp.size();i++)
		{
			int x = corner_coord_temp[i].x;
			int y = corner_coord_temp[i].y;
			std::cout << x << " " << y << std::endl;
		}*/
		
		//Todo: reconsider new coordinate system relative to current marker_coord
		if (marker_found_prev)
		{
			//Reorder corner points for pose estimation algorithm
			for (int i = 0; i < 4; i++)
			{
				if (corner_coord_temp[i].x < frame_size.width / 2 and
					corner_coord_temp[i].y < frame_size.height / 2)
						corner_coord[0] = corner_coord_temp[i];//top left
					
				if (corner_coord_temp[i].x > frame_size.width / 2 and
					corner_coord_temp[i].y < frame_size.height / 2)
						corner_coord[1] = corner_coord_temp[i];//top right
					
				if (corner_coord_temp[i].x < frame_size.width / 2 and
					corner_coord_temp[i].y > frame_size.height / 2)
						corner_coord[2] = corner_coord_temp[i];//bottom left
					
				if (corner_coord_temp[i].x > frame_size.width / 2 and
					corner_coord_temp[i].y > frame_size.height / 2)
						corner_coord[3] = corner_coord_temp[i];//bottom right
			}
		}
			
		/*for (int i = 0; i < corner_coord.size();i++)
		{
			int x = corner_coord[i].x;
			int y = corner_coord[i].y;
			std::cout << x << " " << y << std::endl;
		}*/
    }
    
}

void img_proc::save_img_debug()
{
	for (int i = 0; i < img_array.size(); i ++)
	{
		std::string filename = "image_" + std::to_string(i) + ".jpg";
		imwrite(filename, img_array[i]);
	}
}
