/*
pid class implementation.
*/

#include "pid.h"

pid::pid(double P, double I, double D, double alpha,
			double dt, double upper_bound, double lower_bound,
			bool sat_flag)
{
	this->P = P;
	this->I = I;
	this->D = D;
	this->alpha = alpha;
	this->dt = dt;
	this->upper_bound = upper_bound;
	this->lower_bound = lower_bound;
	this->sat_flag = sat_flag;
}

double pid::eval(double measured, double setpoint)
{
	double error = (setpoint - measured) * alpha + error_prev * (1 - alpha);
    integral += I * error * dt;
    double out = P * error + integral + D * (error - error_prev) / dt;
    if(sat_flag)
    {
    	if(out > upper_bound)
        	out = upper_bound;
        if(out < lower_bound)
            out = lower_bound;
    }
    error_prev = error;
    return out;
}
