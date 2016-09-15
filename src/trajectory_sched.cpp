/*
Reads from file desired trajectory that CF has to follow.
*/

#include "trajectory_sched.h"

trajectory_scheduler::trajectory_scheduler()
{
	std::ifstream reader("trajectory");
	
	reader >> num_rec;
		        
	double _x, _y, _z, _time;
	while(reader >> _time >> _x >> _y >> _z)
	{
		time_vec.push_back(_time);
		x_vec.push_back(_x);
		y_vec.push_back(_y);
		z_vec.push_back(_z);
	}
	
	reader.close();
}

void trajectory_scheduler::get_next_pos(double &x, double &y, double &z)
{
	if(rec_counter < num_rec)
	{
		x = x_vec[rec_counter];
		y = y_vec[rec_counter];
		z = z_vec[rec_counter];
		rec_counter++;
	}
	//get last trajectory point
	else
	{
		x = x_vec[num_rec - 1];
		y = y_vec[num_rec - 1];
		z = z_vec[num_rec - 1];
	}
}

double trajectory_scheduler::get_end_time()
{
	auto it = time_vec.end() - 1;
	return *it;
}

