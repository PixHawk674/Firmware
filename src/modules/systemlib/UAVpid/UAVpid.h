#ifndef _UAVPID
#define _UAVPID

namespace UAVpid
{

class UAVpid
{
public:
	UAVpid(float* input, float* output, float* setpoint, 
	       float kp, float ki, float kd, float ts, 
	       float tau, float upper_limit, float lower_limit);

	bool setOutputLimits(float upper_limit, float lower_limit);

	bool setGains(float kp,float ki,float kd);
	bool reset();
	bool compute();
	float getKp();
	float getKi();
	float getKd();

private:
	float sat(float input, float upper_limit, float lower_limit);
	bool _ready;

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

} //end namespace

#endif