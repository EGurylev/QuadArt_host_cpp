#include "img_proc.h"

img_proc::img_proc()
{
	marker_size = perimeter_prev / 4;
	area_prev = marker_size * marker_size;
	marker_coord.resize(2);
	marker_coord[0] = img_width / 2;
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
    putText(img, to_string(duration), Point2f(500, 500),  FONT_HERSHEY_SIMPLEX, 2.0, Scalar(255,255,255), 2);
	if (cnt < num_debug_img)
		img_array.push_back(img.clone());
	else if (cnt == num_debug_img)//very bad
		save_img_debug();
	cnt += 1;
}

void img_proc::find_marker()
{
	mean_shift(img);
}

void img_proc::track_marker()
{

}

void img_proc::mean_shift(Mat frame)
{
	// Create a new matrix to hold the gray image
	high_resolution_clock::time_point t1 = high_resolution_clock::now();
    Mat gray;
	cvtColor(frame, gray, COLOR_BGR2GRAY);
	Size frame_size = gray.size();
	Mat blurred;
	blur(gray, blurred, Size(10, 10));
	Mat thresholded;
	adaptiveThreshold(blurred, thresholded, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 27,-2);
	vector<vector<Point>> contours;
	findContours(thresholded, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
	
	if (contours.empty())
		return;
	
	vector<double> area, perimeter;
	for (auto cont = contours.begin(); cont != contours.end(); cont++)
	{
		area.push_back(contourArea(*cont));
		perimeter.push_back(arcLength(*cont, true));
	}
	// The etalon marker's area and perimeter comes from previous found marker
	vector<double> diff(contours.size());
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
    
    cout << contours[idx] << endl;
    cout << contours[idx].size() << endl;
    
    //Individual vectors for holding x and y coordinates #todo: make a function
    vector<int> x_coord(cont_min.size()), y_coord(cont_min.size());
    for (int i = 0; i < cont_min.size(); i++)
    {
    	x_coord[i] = cont_min[i].x;
    	y_coord[i] = cont_min[i].y;
    }
    //Coordinates of a marker's center
    marker_coord[0] = accumulate(x_coord.begin(), x_coord.end(), 0) / x_coord.size();
    marker_coord[1] = accumulate(y_coord.begin(), y_coord.end(), 0) / y_coord.size();
    
    /* An alternative approach for calculation center of a contour
    Moments M = moments(cont_min);
	marker_coord[0] = M.m10 / M.m00;
	marker_coord[1] = M.m01 / M.m00;
    */
    
    cout << marker_coord[0] << " " << marker_coord[1] << endl;
	
    //Can be optimized: choose roi around contour of interest 
    find_corners(x_coord, y_coord, frame_size);
	
	high_resolution_clock::time_point t2 = high_resolution_clock::now();
    Duration = duration_cast<microseconds>(t2 - t1).count();
    cout << Duration << endl;
}

void img_proc::find_corners(vector<int>& x, vector<int>& y, Size frame_size)
{
	/*Make Mat object (image) from contour: fill zeros by default
	and ones at contours' points*/
	Mat cont_img;
	cont_img = Mat::zeros(frame_size, CV_32FC1);
	for (int i = 0; i < x.size(); i++)
		cont_img.at<float>(x[i], y[i]) = 1.0;
	//Apply Harris corner detector
	Mat corn_img, bin_img;
	cornerHarris(cont_img, corn_img, 15, 15, 0.1);
	//and make binary image by thresholding
	double min, max;
	minMaxLoc(corn_img, &min, &max);
	threshold(corn_img, bin_img, 0.4 * max, 1, 0);
	//find contours and sort them by area, choose 4 biggest
	bin_img.convertTo(bin_img, CV_8UC1);
	vector<vector<Point>> contours_corn;
	findContours(bin_img, contours_corn, RETR_EXTERNAL, CHAIN_APPROX_NONE);
	
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
    	vector<vector<int>> cont_param;
    	
    	//Iterate through each found contour
    	for (int cont_cnt = 0; cont_cnt < contours_corn.size(); cont_cnt++)
    	{
    		vector<int> x_coord, y_coord;
    		for (int i = 0; i < contours_corn[cont_cnt].size(); i++)
    		{
    			x_coord.push_back(contours_corn[cont_cnt][i].x);
    			y_coord.push_back(contours_corn[cont_cnt][i].y);
    		}
    		//Coordinates of a marker's center
    		int x = accumulate(x_coord.begin(), x_coord.end(), 0) / x_coord.size();
    		int y = accumulate(y_coord.begin(), y_coord.end(), 0) / y_coord.size();
    			
    		vector<int> temp_vec;
    		int area = static_cast<int>(contourArea(contours_corn[cont_cnt]));
    		temp_vec.push_back(area);
    		temp_vec.push_back(x);
    		temp_vec.push_back(y);
    		
    		cont_param.push_back(temp_vec);
    	} 
    		
    	//Sort final cont_param vector by area (0-th element)
    	sort(cont_param.begin(), cont_param.end(),
           	[](const vector<int>& a, const vector<int>& b)
            {
  				return a[0] > b[0];
			});		
		cout << "Sorted vector" << endl;
		for (int i = 0; i < cont_param.size();i++)
		{
			int a,b,c;
			a = cont_param[i][0];
			b = cont_param[i][1];
			c = cont_param[i][2];
			cout << a << " " << b << " " << c << endl;
		}
    }
    
}

void img_proc::save_img_debug()
{
	for (int i = 0; i < img_array.size(); i ++)
	{
		string filename = "image_" + to_string(i) + ".jpg";
		imwrite(filename, img_array[i]);
	}
}
