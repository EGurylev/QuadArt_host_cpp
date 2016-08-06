#include "control_loop.h"

Loop::Loop()
{
	QObject::connect(&loop_timer, SIGNAL(timeout()),
    	this, SLOT(update()));
    img_label.show();
}

void Loop::run()
{
	loop_timer.start(timer_period);
}

void Loop::update()
{
	img = cam_obj.grab_image();
	auto now = std::chrono::system_clock::now();
    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
    auto epoch = now_ms.time_since_epoch();
    auto value = std::chrono::duration_cast<std::chrono::milliseconds>(epoch);
    long duration = value.count();
    putText(img, to_string(duration), Point2f(500, 500),  FONT_HERSHEY_SIMPLEX, 2.0, Scalar(255,255,255), 2);
	if (cnt < num_debug_img)
		img_array.push_back(img.clone());
	cnt += 1;
}

Loop::~Loop()
{
	loop_timer.stop();
	for (int i = 0; i < img_array.size(); i ++)
	{
		string filename = "img_" + to_string(i) + ".jpg";
		imwrite(filename, img_array[i]);
	}
}
