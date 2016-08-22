/*
Control loop implementation.
*/

#include "control_loop.h"
#include <thread>

Loop::Loop() : cf_obj("radio://0/80/250K")
{
	QObject::connect(&loop_timer, SIGNAL(timeout()),
    	this, SLOT(update()));
    img_label.show();
    std::thread log_thread(&Loop::logging, this);
	log_thread.detach();
}

void Loop::run()
{
	loop_timer.start(timer_period);
}

void Loop::logging()
{
	//Recieve telemetry data from crazyflie
	cf_obj.logReset();
	cf_obj.requestLogToc();
	std::function<void(uint32_t, log_data*)> cb =
		std::bind(&Loop::print_log, this, std::placeholders::_1, std::placeholders::_2);
	LogBlock<log_data> stabilizer_log(&cf_obj,
		{{"stabilizer", "roll"},
		{"stabilizer", "pitch"},
		{"stabilizer", "yaw"}}, cb);
	stabilizer_log.start(1);//every 10 ms
    
    //Infinite loop in order to log data
	while(true)
	{
		cf_obj.sendPing();
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}	 
}

void Loop::print_log(uint32_t time_in_ms, log_data* data)
{
	std::cout << data->roll << std::endl;
	std::cout << data->pitch << std::endl;
	std::cout << data->yaw << std::endl;
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
		
	//Send command signals to crazyflie
	if(thrust <= 0)
		thrust = 0;
	else
		thrust -= 200;
	cf_obj.sendSetpoint(0.0, 0.0, 0.0, thrust);
	
}

Loop::~Loop()
{
	loop_timer.stop();
}
