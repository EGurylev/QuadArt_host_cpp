#pragma once

#include "common.h"

class trajectory_scheduler
{
	private:
		int num_rec;
		int rec_counter = 0;
		std::vector<double> time_vec;
		std::vector<double> x_vec;
		std::vector<double> y_vec;
		std::vector<double> z_vec;
	public:
		trajectory_scheduler();
		
		void get_next_pos(double &x, double &y, double &z);
		double get_end_time();
};
