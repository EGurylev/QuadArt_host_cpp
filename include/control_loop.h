/*
Control loop interface
*/

#pragma once

#include <thread>
#include "common.h"
#include "camera.h"
#include "img_proc.h"
#include "pose_estim.h"
#include "Crazyradio.h"
#include "Crazyflie.h"
#include "pid.h"
#include "trajectory_sched.h"
#include "observer.h"

struct log_block
{
	float roll;
	float pitch;
	float yaw;
} __attribute__((packed));

struct control
{
	int thrust = 0;
	double roll = 0;
	double pitch = 0;
	double yaw = 0;
	int thrust_max = 65000;
};

//Structures for holding pid coef's ids
struct cf_pid_coef
{
	float kp;
	float ki;
	float kd;
};
struct cf_pid_param
{
	cf_pid_coef roll;
	cf_pid_coef pitch;
	cf_pid_coef yaw;
};

class Timer
{
	public:
		Timer(double total_time) :
			_execute(true),
			total_time(total_time){}

		bool _execute;
		void start(int interval, std::function<void(void)> func);
		void stop();
		double time = 0;
		double total_time;

	private:
		high_resolution_clock::time_point start_time, end_time;
};

class Loop
{
	private:
		double total_time;//sec
		Camera cam_obj;
		uint8_t* img_p;
		int timer_period = 10000;//microsec
		double time_landing = 2;//sec
		//time before timer ends for shut down
		double time_shutdown = 0.5;
		cv::Mat rvec, tvec;
		img_proc img_proc_obj;
		pose_estimator pe_obj;
		pose6D pose_est, pose_meas, pose_obs;
		std::pair<std::vector<std::string>,
			std::vector<std::vector<double>>> logger;
		Crazyflie cf_obj;
		int thrust_eq = 36000;
		high_resolution_clock::time_point start_time;
		bool is_ready = false;
		int not_valid_count = 0;
		int not_valid_period = 10;
		double x_desired, y_desired, z_desired;
		control control_set;
		pid z_controller;
		pid x_controller;
		pid y_controller;
		observer x_observer;
		cf_pid_param attitude_pid_ids;
		cf_pid_param rate_pid_ids;
		trajectory_scheduler traject;

		void shut_down();
	public:
		Loop();
		~Loop();
		
		Timer timer;

		void feedback_control();
		
		void run();
		void logging();
		void log_callback(uint32_t time_in_ms,
			log_block* data);
		void log2file();
        void update();
};
