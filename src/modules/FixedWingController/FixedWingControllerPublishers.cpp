#include "FixedWingController.h"

namespace FixedWingController
{

void FixedWingController::publish_actuators()
{
	_actuators.timestamp = hrt_absolute_time();
	_actuators_airframe.timestamp = hrt_absolute_time();
	if (_actuators_0_pub > 0) {
		/* publish the attitude setpoint */
		orb_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, &_actuators);

	} else {
		/* advertise and publish */
		_actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators);
	}

	if (_actuators_1_pub > 0) {
		/* publish the attitude setpoint */
		orb_publish(ORB_ID(actuator_controls_1), _actuators_1_pub, &_actuators_airframe);
//				warnx("%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
//						(double)_actuators_airframe.control[0], (double)_actuators_airframe.control[1], (double)_actuators_airframe.control[2],
//						(double)_actuators_airframe.control[3], (double)_actuators_airframe.control[4], (double)_actuators_airframe.control[5],
//						(double)_actuators_airframe.control[6], (double)_actuators_airframe.control[7]);

	} else {
		/* advertise and publish */
		_actuators_1_pub = orb_advertise(ORB_ID(actuator_controls_1), &_actuators_airframe);
	}

}


void FixedWingController::publish_converted_setpoint()
{
	struct vehicle_attitude_setpoint_s att_sp;
	att_sp.timestamp = hrt_absolute_time();
	att_sp.roll_body = _x_command.phi;
	att_sp.pitch_body = _x_command.theta;
	att_sp.yaw_body = 0.0f - _parameters.trim_yaw;
	att_sp.thrust = _delta_t;

	/* lazily publish the setpoint only once available */
	if (_attitude_sp_pub > 0) {
		/* publish the attitude setpoint */
		orb_publish(ORB_ID(vehicle_attitude_setpoint), _attitude_sp_pub, &att_sp);

	} else {
		/* advertise and publish */
		_attitude_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);
	}

}

} // end namespace