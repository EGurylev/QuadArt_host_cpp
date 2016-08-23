/*
Control loop implementation.
*/

#include "control_loop.h"
#include <thread>

Loop::Loop() : cf_obj("radio://0/80/250K")
{
	QObject::connect(&loop_timer, SIGNAL(timeout()),
    	this, SLOT(update()));
    
    logger.first.push_back("roll_cf");
    logger.first.push_back("pitch_cf");
    logger.first.push_back("yaw_cf");
    
    img_label.show();
    std::thread log_thread(&Loop::logging, this);
	log_thread.detach();
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
		
	//Send command signals to crazyflie
	if(thrust <= 0)
		thrust = 0;
	else
		thrust -= 200;
	cf_obj.sendSetpoint(0.0, 0.0, 0.0, thrust);
	
	//Log data
	std::vector<float> log_slice;
	log_slice.push_back(pose_meas.roll);
	log_slice.push_back(pose_meas.pitch);
	log_slice.push_back(pose_meas.yaw);
	logger.second.push_back(log_slice);
	
}

void Loop::logging()
{
	//Recieve telemetry data from crazyflie
	cf_obj.logReset();
	cf_obj.requestLogToc();
	std::function<void(uint32_t, log_block*)> cb =
		std::bind(&Loop::log_callback, this,
		std::placeholders::_1, std::placeholders::_2);
	LogBlock<log_block> stabilizer_log(&cf_obj,
		{{"stabilizer", "roll"},
		{"stabilizer", "pitch"},
		{"stabilizer", "yaw"}}, cb);
	stabilizer_log.start(1);//every 10 ms
    
    //Infinite loop in order to log data
	while(true)
	{
		cf_obj.sendPing();
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}	 
}

void Loop::log_callback(uint32_t time_in_ms, log_block* data)
{
	pose_meas.roll = data->roll;
	pose_meas.pitch = data->pitch;
	pose_meas.yaw = data->yaw;
}

void Loop::log2file()
{
	auto end = std::chrono::system_clock::now();
	auto end_time = std::chrono::system_clock::to_time_t(end);
	auto datetime = std::string(std::ctime(&end_time));
	//remove null terminated char
	datetime = datetime.substr(0, datetime.size() - 1);
	auto filename = "Log_" + datetime + ".csv";
	std::ofstream writer(filename);
	
	int num_slices = logger.second.size();
	int num_log_vars = logger.first.size();
	//Print header
	writer << num_slices << "," << num_log_vars << "\n";
	for(int i = 0; i < num_log_vars; i++)
	{
		if(i != num_log_vars - 1)
			writer << logger.first[i] << ",";
		else
			writer << logger.first[i] << "\n";
	}
	//Print log matrix by time slice
	for(int i = 0; i < num_slices; i++)
	{
		for(int j = 0; j < num_log_vars; j++)
		{
			if(j != num_log_vars - 1)
				writer << logger.second[i][j] << ",";
			else
				writer << logger.second[i][j] << "\n";
		}
	}
	
	writer.close();
}

Loop::~Loop()
{
	loop_timer.stop();
	log2file();
}
