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
		double x_final, y_final, z_final;
	public:
		trajectory_scheduler();
		bool is_landing = false;
		bool is_final_pos_set = false;
		void get_next_pos(double &x, double &y, double &z);
		double get_end_time();
		void set_final_position(double &x, double &y, double &z);
};
