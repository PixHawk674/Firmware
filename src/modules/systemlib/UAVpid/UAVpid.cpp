#include "UAVpid.h"

namespace UAVpid
{
UAVpid::UAVpid(float* input, float* output, float* setpoint, 
	 float kp, float ki, float kd, float upper_limit, 
	 float lower_limit, float ts, float tau)
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
	bool _ready = 1;
}


UAVpid::UAVpid()
{
	bool _ready = 0;
}


bool UAVpid::setOutputLimits(float upper_limit, float lower_limit)
{
	_upperLimit = upper_limit;
	_lowerLimit = lower_limit;
	return 1;
}

bool UAVpid::setGains(float kp,float ki,float kd)
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
	if(_ready)
	{
		float t = hrt_absolute_time();
		if(_t_dl - t  < _ts)
		{
			// do nothing
			return 0;
		}
		else
		{
			float error = *_setpoint - *_input;
			_integrator += (_ts/2)*(error+_prev_dl);
			_differentiator = (2*_tau-_ts)/(2*_tau+_ts)*_differentiator+2/(2*_tau+_ts)*(error-_error_dl);
			_error_dl = _error;

			u = sat(_kp*error+_ki*_integrator+_kd*_differentiator, _upperLimit, _lowerLimit);
			// integrator anti-windup scheme
			if(_ki!=0)
			{
				u_unsat = _kp*error+_ki*_integrator+_kd*_differentiator;
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
}

float UAVpid::sat(float input, float upper_limit, float lower_limit)
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

float UAVpid::getKp(){return _kp;}
float UAVpid::getKi(){return _ki;}
float UAVpid::getKd(){return _kd;}
} // end namespace