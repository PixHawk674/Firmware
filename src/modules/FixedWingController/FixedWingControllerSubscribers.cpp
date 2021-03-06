#include "FixedWingController.h"

namespace FixedWingController
{

void
FixedWingController::vehicle_control_mode_poll()
{
	bool vcontrol_mode_updated;

	/* Check HIL state if vehicle status has changed */
	orb_check(_vcontrol_mode_sub, &vcontrol_mode_updated);

	if (vcontrol_mode_updated) {

		orb_copy(ORB_ID(vehicle_control_mode), _vcontrol_mode_sub, &_vcontrol_mode);
	}

}

void
FixedWingController::vehicle_manual_poll()
{
	bool manual_updated;

	/* get pilots inputs */
	orb_check(_manual_sub, &manual_updated);

	if (manual_updated) {

		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
	}
}

void
FixedWingController::vehicle_airspeed_poll()
{
	/* check if there is a new position */
	bool airspeed_updated;
	orb_check(_airspeed_sub, &airspeed_updated);

	if (airspeed_updated) {
		orb_copy(ORB_ID(airspeed), _airspeed_sub, &_airspeed);
//		warnx("airspeed poll: ind: %.4f,  true: %.4f", _airspeed.indicated_airspeed_m_s, _airspeed.true_airspeed_m_s);
	}

	/* if airspeed is not updating, we assume the normal average speed */
	if (bool nonfinite = !isfinite(_airspeed.true_airspeed_m_s) ||
	    hrt_elapsed_time(&_airspeed.timestamp) > 1e6) {
		_airspeed.true_airspeed_m_s = _parameters.airspeed_trim;
		if (nonfinite) {
			perf_count(_nonfinite_input_perf);
		}
	} else {
		/* prevent numerical drama by requiring 0.5 m/s minimal speed */
		_airspeed.true_airspeed_m_s = math::max(0.5f, _airspeed.true_airspeed_m_s);
	}
}

void
FixedWingController::vehicle_accel_poll()
{
	/* check if there is a new position */
	bool accel_updated;
	orb_check(_accel_sub, &accel_updated);

	if (accel_updated) {
		orb_copy(ORB_ID(sensor_accel0), _accel_sub, &_accel);
	}
}

void
FixedWingController::vehicle_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool att_sp_updated;
	orb_check(_att_sp_sub, &att_sp_updated);

	if (att_sp_updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _att_sp_sub, &_att_sp);
		_setpoint_valid = true;
	}
}

void
FixedWingController::global_pos_poll()
{
	/* check if there is a new global position */
	bool global_pos_updated;
	orb_check(_global_pos_sub, &global_pos_updated);

	if (global_pos_updated) {
		orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);
	}
}

void
FixedWingController::vehicle_status_poll()
{
	/* check if there is new status information */
	bool vehicle_status_updated;
	orb_check(_vehicle_status_sub, &vehicle_status_updated);

	if (vehicle_status_updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);
	}
}

void
FixedWingController::x_command_poll()
{

	/*
	 * in manual mode no external source should / does emit attitude setpoints.
	 * emit the manual setpoint here to allow attitude controller tuning
	 * in attitude control mode.
	 */
	if(_vcontrol_mode.flag_control_manual_enabled)
	{
		_x_command.timestamp    = hrt_absolute_time();
		_x_command.theta 	= _att_sp.pitch_body;
		_x_command.phi  	= _att_sp.roll_body;
		_x_command.psi 		= _att_sp.yaw_body;

		// there is a problem here, where _att_sp is not being updated
	}
	else
	{
		//Poll the attitude setpoint (roll, pitch, yaw)
		vehicle_setpoint_poll();

		//Poll the current waypoint information to get altitude and airspeed commands
		bool pos_triplet_updated;
		orb_check(_pos_sp_triplet_sub, &pos_triplet_updated);

		if (pos_triplet_updated) {
			orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);
		}

		_x_command.timestamp 	= _att_sp.timestamp;
		_x_command.theta 	= _att_sp.pitch_body + _parameters.pitchsp_offset_rad;
		_x_command.phi  	= _att_sp.roll_body + _parameters.rollsp_offset_rad;
		_x_command.psi 		= _att_sp.yaw_body;
	}

		float u, v, w;
		u = _pos_sp_triplet.current.vx;
		v = _pos_sp_triplet.current.vy;
		w = _pos_sp_triplet.current.vz;
		_x_command.Va 		= sqrtf(u*u + v*v + w*w);

		_x_command.pn 		= _pos_sp_triplet.current.x;
		_x_command.pe 		= _pos_sp_triplet.current.y;
		_x_command.h 		= _pos_sp_triplet.current.z;

	//_x_command.altitude 	= _pos_sp_triplet.current.alt;
}

void
FixedWingController::x_hat_poll()
{
	bool x_hat_updated;
	orb_check(_x_hat_sub, &x_hat_updated);

	if(x_hat_updated)
	{
		orb_copy(ORB_ID(x_hat), _x_hat_sub, &_x_hat);
	}
}

}// end namespace