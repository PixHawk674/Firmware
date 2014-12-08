#include "FixedWingController.h"

namespace FixedWingController
{

void
FixedWingController::compute_control()
{

	/* If the aircraft is on ground reset the integrators */
	if (_vehicle_status.condition_landed) 
	{
		_rollControl->reset();
		_courseControl->reset();
		_airspeedPitchHold_ctrl->reset();
		_airspeedThrottleHold_ctrl->reset();
		_altitudeHold_ctrl->reset();
	}

	/* if lock integrator is active, set ki to zero 
	if (_lock_integrator)
	{
		// to be implemented later
	}
	*/

	/* Run Longitudinal Controller */
	float h = _x_hat.h;
	float h_c = -1.0f*_x_command.h;

	if(h < _parameters.altitude_takeoff_zone)
	{
		//In take-off zone
		_delta_t = 1.0f;
		_x_command.theta = _parameters.theta_takeoff*3.1415f/180.0f;
	}
	else if(h <= h_c - _parameters.altitude_hold_zone)
	{
		//In Climb Zone
		_delta_t = 1;
		_airspeedPitchHold_ctrl->compute();
	}
	else if(h >= h_c + _parameters.altitude_hold_zone)
	{
		//In Descend Zone
		_delta_t = 0;
		_airspeedPitchHold_ctrl->compute();
	}
	else
	{
		//In Altitude-Hold Zone
		_airspeedThrottleHold_ctrl->compute();
		_altitudeHold_ctrl->compute();
	}
	_pitchHold_ctrl->compute();

	/* Run Lateral Controller */
	_courseControl->compute();
	_rollControl->compute();
	_delta_r = 0; // for now, no slideslip controller
}


} // end namespace