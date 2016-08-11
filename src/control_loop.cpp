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
	//Get image from camera
	img_p = cam_obj.grab_image();
	
	//Get marker coordinates from image in 2D
	img_proc_obj.marker_search(img_p);
	
	//Get rotation and translation vector of 
	//marker in camera coordinate system
	pe_obj.calc_pose(img_proc_obj.corner_coord,
		rvec, tvec);
		
	cout << tvec << endl;
}

Loop::~Loop()
{
	loop_timer.stop();
}
