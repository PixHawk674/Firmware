#include "UAVpid.h"

namespace UAVpid_UAV
{
UAVpid::UAVpid(double* input, double* output, double* setpoint, 
	 double kp, double ki, double kd, double upper_limit, 
	 double lower_limit, double ts, double tau)
{
	_output   = output;
	_input    = input;
	_setpoint = setpoint;
	_tau      = tau;
	_ts       = ts;
	_t_dl	  = hrt_absolute_time();

	setOutputLimits(upper_limit, lower_limit);
	setGains(kp,ki,kd);
	reset();


}

bool UAVpid::setOutputLimits(double upper_limit, double lower_limit)
{
	_upperLimit = upper_limit;
	_lowerLimit = lower_limit;
	return 1;
}

bool UAVpid::setGains(kp,ki,kd)
{
	_kp = kp;
	_ki = ki;
	_kd = kd;
	return 1;
}

bool UAVpid::reset()
{
	_integrator = 0;
	_differentiator = 0;
	_error_dl = 0;
	return 1;
}

bool UAVpid::compute()
{
	double t = hrt_absolute_time();
	if(_t_dl - t  < _ts)
	{
		// do nothing
		return 0;
	}
	else
	{
		double error = *_setpoint - *_input;
		_integrator += (_ts/2)*(error+_prev_dl);
		_differentiator = (2*_tau-_ts)/(2*_tau+_ts)*_differentiator+2/(2*_tau+_ts)*(error-_error_dl);
		_error_dl = _error;

		u = sat(kp*error+ki*_integrator+kd*_differentiator, _upperLimit, _lowerLimit);
		// integrator anti-windup scheme
		if(_ki!=0)
		{
			u_unsat = kp*error+ki*_integrator+kd*_differentiator;
			if(u!=u_unsat)
			{
				_integrator = _integrator + _ts/_ki*(u-u_unsat);
			}
		}
		*_output = u;
		_t_dl = hrt_absolute_time();
		return 1;
	}	
}

double UAVpid::sat(double input, double upper_limit, double lower_limit)
{
	if(u>upper_limit)
	{
		u = upper_limit;
	}
	if(u<lower_limit)
	{
		u = lower_limit;
	}
	return u;
}

double UAVpid::getKp(){return _kp;}
double UAVpid::getKi(){return _ki;}
double UAVpid::getKd(){return _kd;}
} // end namespace