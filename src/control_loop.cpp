/*
Control loop implementation. Method update()
runs periodically and invokes next tasks:
	1. Image grabbing from camera.
	2. Get marker during image processing.
	3. Position calculation.
	4. Calculate control signals and send them into quadrotor.
	5. Log parameters.
*/

#include "control_loop.h"

Loop::Loop() :

	log_thread(&Loop::logging, this),

	cf_obj("radio://0/80/250K"),
	
	z_controller(260, 120, 200, 0.6,
		timer_period / 1e6, 17000,
		-17000, true),
		
	x_controller(0.6, 0.15, 0.4, 0.8,
		timer_period / 1e6, 25,
		-25, true),
		
	y_controller(0.2, 0.05, 0.15, 0.5,
		timer_period / 1e6, 25,
		-25, true),
		
	x_observer("x_observer"),
	
	y_observer("y_observer"),
	
	z_observer("z_observer"),
		
	timer(traject.get_end_time() + time_landing)
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
    logger.first.push_back("x_set");
    logger.first.push_back("y_set");
    logger.first.push_back("z_set");
    logger.first.push_back("x_obs");
    logger.first.push_back("y_obs");
    logger.first.push_back("z_obs");
    logger.first.push_back("proj_error");
    logger.first.push_back("is_pose_valid");
    
    //Set ids for CF pid coefs
    rate_pid_ids.roll.kp = 14;
    rate_pid_ids.roll.ki = 15;
    rate_pid_ids.roll.kd = 16;
    rate_pid_ids.pitch.kp = 17;
    rate_pid_ids.pitch.ki = 18;
    rate_pid_ids.pitch.kd = 19;
    rate_pid_ids.yaw.kp = 20;
    rate_pid_ids.yaw.ki = 21;
    rate_pid_ids.yaw.kd = 22;
    attitude_pid_ids.roll.kp = 23;
    attitude_pid_ids.roll.ki = 24;
    attitude_pid_ids.roll.kd = 25;
    attitude_pid_ids.pitch.kp = 26;
    attitude_pid_ids.pitch.ki = 27;
    attitude_pid_ids.pitch.kd = 28;
    attitude_pid_ids.yaw.kp = 29;
    attitude_pid_ids.yaw.ki = 30;
    attitude_pid_ids.yaw.kd = 31;
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
	
	//State observer (separate channels for x, y and z)
	pose_obs.x = x_observer.update(pose_est.x, control_set.pitch, pose_est.isvalid);
	pose_obs.y = y_observer.update(pose_est.y, control_set.roll, pose_est.isvalid);
	//Observer for z position has to account equilibrium command signal. It must
	//be took off from command signal if it is greater than zero (flying mode)
	if (control_set.thrust > 0)
		pose_obs.z = z_observer.update(pose_est.z, control_set.thrust - thrust_eq, pose_est.isvalid);
	else
		pose_obs.z = z_observer.update(pose_est.z, control_set.thrust, pose_est.isvalid);

	//Feedback control
	feedback_control();
	//Send command signals to crazyflie
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
	log_slice.push_back(x_desired);
	log_slice.push_back(y_desired);
	log_slice.push_back(z_desired);
	log_slice.push_back(pose_obs.x);
	log_slice.push_back(pose_obs.y);
	log_slice.push_back(pose_obs.z);
	log_slice.push_back(pe_obj.log_debug.proj_error);
	log_slice.push_back(pose_est.isvalid);
	logger.second.push_back(log_slice);
	
}

void Loop::feedback_control()
{
	//Shutdown during landing
	if (traject.is_landing && (timer.total_time - timer.time) < time_shutdown)
	{
		shut_down();
	}
	//Flight mode
	else if(pose_est.isvalid && is_ready)
	{
		//Set position for landing as it was at launch
		if (!traject.is_final_pos_set)
		{
			traject.set_final_position(pose_est.x, pose_est.y, pose_est.z);
		}

		//Get trajectory points
		traject.get_next_pos(x_desired, y_desired, z_desired);
	
		control_set.thrust = 
			static_cast<int>(z_controller.eval(pose_obs.z, z_desired)) +
			thrust_eq;
		//Clip thrust to valid range
		if(control_set.thrust > control_set.thrust_max)
			control_set.thrust = control_set.thrust_max;
		if(control_set.thrust < 0)
			control_set.thrust = 0;
			
		control_set.pitch = -x_controller.eval(pose_obs.x, x_desired);
		control_set.roll = -y_controller.eval(pose_obs.y, y_desired);
		control_set.yaw = 0;
		not_valid_count = 0;
	}
	//Shutdown if marker isn't found
	else
	{
		//Shut down if pose estimation is not valid
		//during specified period measured in cycles
		not_valid_count++;
		if(not_valid_count >= not_valid_period)
		{
			shut_down();
		}
	}
}

void Loop::shut_down()
{
	control_set.thrust = 0;
	control_set.pitch = 0;
	control_set.roll = 0;
	control_set.yaw = 0;
	std::cout << "Shut down!" << std::endl;
}

void Loop::logging()
{
	//Recieve telemetry data from crazyflie
	cf_obj.logReset();
	cf_obj.requestLogToc();
	//Set pid parameters
	cf_obj.requestParamToc();
	//cf_obj.setParam(attitude_pid_ids.pitch.kp, 1);//this doesn't set param. Why?
	//cf_obj.setParam(rate_pid_ids.pitch.kp, 50);
	//Now CF is ready for control signals
	is_ready = true;
	//Init time must be added to timer
	high_resolution_clock::time_point time_now =
		high_resolution_clock::now();
	double time_init = static_cast<double>(duration_cast<microseconds>
		(time_now - start_time).count());
	timer.total_time += time_init / 1e6;

	std::function<void(uint32_t, log_block*)> cb =
		std::bind(&Loop::log_callback, this,
		std::placeholders::_1, std::placeholders::_2);
	LogBlock<log_block> stabilizer_log(&cf_obj,
		{{"stabilizer", "roll"},
		{"stabilizer", "pitch"},
		{"stabilizer", "yaw"}}, cb);
	stabilizer_log.start(1);//every 10 ms
    
    //Infinite loop in order to log data
	while(!stop_thread)
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
	stop_thread = true;
	log_thread.join();
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
			int sleep_period;
        	if(dt < interval)
				sleep_period = interval - dt;
			else
				sleep_period = 0;
			std::this_thread::sleep_for(
				std::chrono::microseconds(sleep_period));
			time += (dt + sleep_period) / 1e6;
			if(time > total_time)
				_execute = false;
        }
	}).detach();
}
