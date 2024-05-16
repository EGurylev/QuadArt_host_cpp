/*
Class for proportional-integral-derivative
controller with saturation and wind up.
It implements parallel pid controller.
*/
#pragma once

#include <limits>

class pid
{
	private:
		double P = 0;
		double I = 0;
		double D = 0;
		double dt;
		double integral = 0;
		double error_prev = 0;
		double alpha = 0;
		double upper_bound = 
			std::numeric_limits<double>::infinity();
		double lower_bound = 
			-std::numeric_limits<double>::infinity();
		bool sat_flag = false;
	public:
		pid(double P, double I, double D, double alpha,
			double dt, double upper_bound, double lower_bound,
			bool sat_flag);
		double eval(double measured, double setpoint);
};
