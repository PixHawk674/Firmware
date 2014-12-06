#include <FixedWingController.h>

namespace FixedWingController
{

FixedWingController::compute_lateral_control()
{
	airspeed_scaling = _parameters.airspeed_trim / ((airspeed < _parameters.airspeed_min) ? _parameters.airspeed_min : airspeed);

	/* If the aircraft is on ground reset the integrators */
	if (_vehicle_status.condition_landed) {
		_rollControl.reset();
		_courseControl.reset();
	}

	// if in manual mode, don't run course control,
	if(!_vcontrol_mode.flag_control_manual_enabled)
	{
		// compute the lateral controller
		_courseControl.compute();  // course control writes to _x_command_phi, which is 
	}
	_rollControl.compute();	
}


} // end namespace