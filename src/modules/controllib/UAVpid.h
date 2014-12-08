#ifndef _UAVPID
#define _UAVPID

#include <drivers/drv_hrt.h>

class UAVpid
{
public:
	UAVpid(float* input, float* output, float* setpoint, 
	       float kp, float ki, float kd, float ts, 
	       float tau, float upper_limit, float lower_limit)
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
		_readyToCompute = 1;
	}

	UAVpid()
	{
		_readyToCompute = 0;
	}

	bool setOutputLimits(float upper_limit, float lower_limit)
	{
		_upperLimit = upper_limit;
		_lowerLimit = lower_limit;
		return 1;
	}

	bool setGains(float kp,float ki,float kd)
	{
		_kp = kp;
		_ki = ki;
		_kd = kd;
		return 1;
	}


	bool reset()
	{
		_integrator = 0.0f;
		_differentiator = 0.0f;
		_error_dl = 0.0f;
		return 1;
	}

	bool compute()
	{
		if(_readyToCompute)
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
				_integrator += (_ts/2.0f)*(error+_error_dl);
				_differentiator = (2.0f*_tau-_ts)/(2.0f*_tau+_ts)*_differentiator+2.0f/(2.0f*_tau+_ts)*(error-_error_dl);
				_error_dl = error;

				float u = sat(_kp*error+_ki*_integrator+_kd*_differentiator, _upperLimit, _lowerLimit);
				// integrator anti-windup scheme
				if(_ki!=0.0f)
				{
					float u_unsat = _kp*error+_ki*_integrator+_kd*_differentiator;
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

	float getKp(){return _kp;}
	float getKi(){return _ki;}
	float getKd(){return _kd;}

private:
	float sat(float input, float upper_limit, float lower_limit)
	{
		float u = input;
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
	
	bool _readyToCompute;

	float* _output;
	float* _input;
	float* _setpoint;
	float  _tau;
	float  _ts;
	float  _t_dl;
	
	float  _upperLimit;
	float  _lowerLimit;

	float  _kp;
	float  _ki;
	float  _kd;

	float  _integrator;
	float  _differentiator;
	float  _error_dl;

};

#endif