/*
Control loop implementation.
*/

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
	img_p = cam_obj.grab_image();
	img_proc_obj.marker_search(img_p);
}

Loop::~Loop()
{
	loop_timer.stop();
}
