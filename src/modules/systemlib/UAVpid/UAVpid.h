namespace UAVpid
{

class UAVpid
{
public:
	UAVpid(double* input, double* output, double* setpoint, 
	       double kp, double ki, double kd, double ts, 
	       double tau, double upper_limit, double lower_limit);

	bool setOutputLimits(double upper_limit, double lower_limit);

	bool setGains(kp,ki,kd);
	bool reset();
	bool compute();
	double getKp();
	double getKi();
	double getKd();

private:
	double sat(double input, double upper_limit, double lower_limit);

	double* _output;
	double* _input;
	double* _setpoint;
	double  _tau;
	double  _ts;
	double  _t_dl;
	
	double  _upperLimit;
	double  _lowerLimit;

	double  _kp;
	double 	_ki;
	double	_kp;

	double 	_integrator;
	double	_differentiator;
	double	_error_dl;

};

} //end namespace