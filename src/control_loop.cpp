/*
Control loop implementation.
*/

#include "control_loop.h"
#include <thread>

Loop::Loop() :
	cf_obj("radio://0/80/250K"),
	
	z_controller(80, 30, 90, 0.35,
		timer_period / 1000.0, 7000,
		-7000, true),
		
	x_controller(0.1, 0.01, 0.12, 0.15,
		timer_period / 1000.0, 20,
		-20, true),
		
	y_controller(0.1, 0.01, 0.12, 0.15,
		timer_period / 1000.0, 20,
		-20, true)
{
	QObject::connect(&loop_timer, SIGNAL(timeout()),
    	this, SLOT(update()));
    //Log variables
    logger.first.push_back("marker_found");
    logger.first.push_back("roll_cf");
    logger.first.push_back("pitch_cf");
    logger.first.push_back("yaw_cf");
    logger.first.push_back("x");
    logger.first.push_back("y");
    logger.first.push_back("z");
    
    //Initialize pid controllers
	
    
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
	marker* Marker = img_proc_obj.marker_search(img_p);
	
	//Get rotation and translation vector of 
	//marker in camera coordinate system
	pe_obj.calc_pose(Marker, pose_est);
	
	//Feedback control
	double thrust_set, roll_set, pitch_set, yaw_set;
	feedback_control(thrust_set, roll_set,
			pitch_set, yaw_set);
	//Send command signals to crazyflie
	if(thrust_eq <= 0)
		thrust_eq = 0;
	else
		thrust_eq -= 200;
	cf_obj.sendSetpoint(0.0, 0.0, 0.0, thrust_eq);
	
	//Log data
	std::vector<double> log_slice;
	log_slice.push_back(Marker->found);
	log_slice.push_back(pose_meas.roll);
	log_slice.push_back(pose_meas.pitch);
	log_slice.push_back(pose_meas.yaw);
	log_slice.push_back(pose_est.x);
	log_slice.push_back(pose_est.y);
	log_slice.push_back(pose_est.z);
	logger.second.push_back(log_slice);
	
}

void Loop::feedback_control
			(double &thrust_set, double &roll_set,
			 double &pitch_set, double &yaw_set)
{
	if(pose_est.isvalid)
	{
		thrust_set = z_controller.eval(-pose_est.z, 0);
		thrust_set += thrust_eq;
		pitch_set = x_controller.eval(pose_est.x, 0);
		roll_set = -y_controller.eval(pose_est.y, 0);
		yaw_set = 0;
	}
	else
	{
		thrust_set = 0;
		pitch_set = 0;
		roll_set = 0;
		yaw_set = 0;
	}
	thrust_set = static_cast<int>(thrust_set);
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
		std::this_thread::sleep_for(std::chrono::milliseconds(2));
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
