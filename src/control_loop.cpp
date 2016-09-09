/*
Control loop implementation.
*/

#include "control_loop.h"

Loop::Loop() :
	cf_obj("radio://0/80/250K"),
	
	z_controller(80, 30, 70, 0.3,
		timer_period / 1e6, 15000,
		-15000, true),
		
	x_controller(0.2, 0.05, 0.06, 0.15,
		timer_period / 1e6, 20,
		-20, true),
		
	y_controller(0.2, 0.05, 0.05, 0.15,
		timer_period / 1e6, 20,
		-20, true),
		
	timer(total_time)
{
	//Init start time
	start_time = high_resolution_clock::now();
	
    //Log variables
    logger.first.push_back("time");
    logger.first.push_back("marker_found");
    logger.first.push_back("thrust_set");
    logger.first.push_back("pitch_set");
    logger.first.push_back("roll_set");
    logger.first.push_back("roll_cf");
    logger.first.push_back("pitch_cf");
    logger.first.push_back("yaw_cf");
    logger.first.push_back("time_cf");
    logger.first.push_back("x");
    logger.first.push_back("y");
    logger.first.push_back("z");
    logger.first.push_back("proj_error");
    logger.first.push_back("is_pose_valid");
    
    //Run telemetry data logging in a separate thread
    std::thread log_thread(&Loop::logging, this);
	log_thread.detach();
}

void Loop::run()
{
	timer.start(timer_period, std::bind(&Loop::update, this));
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
	feedback_control();
	//Send command signals to crazyflie
	/*if(thrust_eq <= 0)
		thrust_eq = 0;
	else
		thrust_eq -= 200;*/
	cf_obj.sendSetpoint(control_set.roll, control_set.pitch,
		control_set.yaw, control_set.thrust);
	
	//Measure current time in ms
	high_resolution_clock::time_point time_now = 
		high_resolution_clock::now();
	double log_time = static_cast<double>(duration_cast<microseconds>
		(time_now - start_time).count());
	
	//Log data
	std::vector<double> log_slice;
	log_slice.push_back(log_time);
	log_slice.push_back(Marker->found);
	log_slice.push_back(control_set.thrust);
	log_slice.push_back(control_set.pitch);
	log_slice.push_back(control_set.roll);
	log_slice.push_back(pose_meas.roll);
	log_slice.push_back(pose_meas.pitch);
	log_slice.push_back(pose_meas.yaw);
	log_slice.push_back(pose_meas.time_stamp);
	log_slice.push_back(pose_est.x);
	log_slice.push_back(pose_est.y);
	log_slice.push_back(pose_est.z);
	log_slice.push_back(pe_obj.log_debug.proj_error);
	log_slice.push_back(pose_est.isvalid);
	logger.second.push_back(log_slice);
	
}

void Loop::feedback_control()
{
	if(pose_est.isvalid)
	{
		control_set.thrust = 
			static_cast<int>(z_controller.eval(pose_est.z, 20)) +
			thrust_eq;
		//Clip thrust to valid range
		if(control_set.thrust > control_set.thrust_max)
			control_set.thrust = control_set.thrust_max;
		if(control_set.thrust < 0)
			control_set.thrust = 0;
			
		control_set.pitch = -x_controller.eval(pose_est.x, 0);
		control_set.roll = -y_controller.eval(pose_est.y, 80);
		control_set.yaw = 0;
		not_valid_count = 0;
	}
	else
	{
		//Shut down if pose estimation is not valid
		//during specified period measured in cycles
		not_valid_count++;
		if(not_valid_count >= not_valid_period)
		{
			control_set.thrust = 0;
			control_set.pitch = 0;
			control_set.roll = 0;
			control_set.yaw = 0;
		}
	}
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
	pose_meas.time_stamp = time_in_ms;
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
	log2file();
}

void Timer::start(int interval, std::function<void(void)> func)
{
	std::thread([=]()
    {
    	while (_execute)
    	{
    		start_time = high_resolution_clock::now();
        	func();
        	end_time = high_resolution_clock::now();
        	int dt = static_cast<int>(duration_cast<microseconds>
        		(end_time - start_time).count());
        	int period;
        	if(dt < interval)
        		period = interval - dt; 
        	else
        		period = 0; 
        	std::this_thread::sleep_for(
        		std::chrono::microseconds(period));
        	time += 0.01;
        	if(time > _total_time)
        		_execute = false;
        }
	}).detach();
}
