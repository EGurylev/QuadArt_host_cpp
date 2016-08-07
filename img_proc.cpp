#include "img_proc.h"

void img_proc::marker_search(uint8_t* input_img)
{
	//Convert to OpenCV format
	high_resolution_clock::time_point t1 = high_resolution_clock::now();
	Mat img = cv::Mat(img_height, img_width, CV_8UC3, input_img);
	high_resolution_clock::time_point t2 = high_resolution_clock::now();
    Duration = duration_cast<microseconds>(t2 - t1).count();
    //cout << Duration << endl;
    auto now = std::chrono::system_clock::now();
    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
    auto epoch = now_ms.time_since_epoch();
    auto value = std::chrono::duration_cast<std::chrono::milliseconds>(epoch);
    long duration = value.count();
    putText(img, to_string(duration), Point2f(500, 500),  FONT_HERSHEY_SIMPLEX, 2.0, Scalar(255,255,255), 2);
	if (cnt < num_debug_img)
		img_array.push_back(img.clone());
	else if (cnt == num_debug_img)//very bad
		this->save_img_debug();
	cnt += 1;
}

void img_proc::save_img_debug()
{
	for (int i = 0; i < img_array.size(); i ++)
	{
		string filename = "image_" + to_string(i) + ".jpg";
		imwrite(filename, img_array[i]);
	}
}
