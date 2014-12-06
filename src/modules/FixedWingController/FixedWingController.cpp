/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file fw_att_control_main.c
 * Implementation of a generic attitude controller based on classic orthogonal PIDs.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 * @author James Jackson <james.s.jackson@byu.edu>
 *
 */

#include "fwMAGICCLateralControl.h"

namespace LateralControl
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

FixedWingController	*g_control = nullptr;

FixedWingController::FixedWingController() :

	_task_should_exit(false),
	_task_running(false),
	_control_task(-1),

/* subscriptions */
	_att_sub(-1),
	_accel_sub(-1),
	_airspeed_sub(-1),
	_vcontrol_mode_sub(-1),
	_params_sub(-1),
	_manual_sub(-1),
	_global_pos_sub(-1),
	_vehicle_status_sub(-1),

/* publications */
	_rate_sp_pub(-1),
	_attitude_sp_pub(-1),
	_actuators_0_pub(-1),
	_actuators_1_pub(-1),

/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "fw att control")),
	_nonfinite_input_perf(perf_alloc(PC_COUNT, "fw att control nonfinite input")),
	_nonfinite_output_perf(perf_alloc(PC_COUNT, "fw att control nonfinite output")),
/* states */
	_setpoint_valid(false),
	_debug(false)
{
	/* safely initialize structs */
	_att = {};
	_accel = {};
	_att_sp = {};
	_manual = {};
	_airspeed = {};
	_vcontrol_mode = {};
	_actuators = {};
	_actuators_airframe = {};
	_global_pos = {};
	_vehicle_status = {};

	/* MAGICC Params Added */
	_parameter_handles.kp_roll = param_find("FW_ROLL_P");
	_parameter_handles.ki_roll = param_find("FW_ROLL_I");
	_parameter_handles.kd_roll = param_find("FW_ROLL_D");
	_parameter_handles.max_roll_output = param_find("FW_MAX_AILERON_OUTPUT");
	_parameter_handles.max_roll_output = param_find("FW_MIN_AILERON_OUTPUT");
	_parameter_handles.kp_course = param_find("FW_COURSE_P");
	_parameter_handles.ki_course = param_find("FW_COURSE_I");
	_parameter_handles.kd_course = param_find("FW_COURSE_D");
	_parameter_handles.max_course_output = param_find("FW_MAX_COURSE_OUTPUT");
	_parameter_handles.tau = param_find("FW_TAU");

	/* original parameters */
	_parameter_handles.tconst = param_find("FW_ATT_TC");
	_parameter_handles.p_p = param_find("FW_PR_P");
	_parameter_handles.p_i = param_find("FW_PR_I");
	_parameter_handles.p_ff = param_find("FW_PR_FF");
	_parameter_handles.p_rmax_pos = param_find("FW_P_RMAX_POS");
	_parameter_handles.p_rmax_neg = param_find("FW_P_RMAX_NEG");
	_parameter_handles.p_integrator_max = param_find("FW_PR_IMAX");
	_parameter_handles.p_roll_feedforward = param_find("FW_P_ROLLFF");

	_parameter_handles.r_p = param_find("FW_RR_P");
	_parameter_handles.r_i = param_find("FW_RR_I");
	_parameter_handles.r_ff = param_find("FW_RR_FF");
	_parameter_handles.r_integrator_max = param_find("FW_RR_IMAX");
	_parameter_handles.r_rmax = param_find("FW_R_RMAX");

	_parameter_handles.y_p = param_find("FW_YR_P");
	_parameter_handles.y_i = param_find("FW_YR_I");
	_parameter_handles.y_ff = param_find("FW_YR_FF");
	_parameter_handles.y_integrator_max = param_find("FW_YR_IMAX");
	_parameter_handles.y_rmax = param_find("FW_Y_RMAX");

	_parameter_handles.airspeed_min = param_find("FW_AIRSPD_MIN");
	_parameter_handles.airspeed_trim = param_find("FW_AIRSPD_TRIM");
	_parameter_handles.airspeed_max = param_find("FW_AIRSPD_MAX");

	_parameter_handles.y_coordinated_min_speed = param_find("FW_YCO_VMIN");

	_parameter_handles.trim_roll = param_find("TRIM_ROLL");
	_parameter_handles.trim_pitch = param_find("TRIM_PITCH");
	_parameter_handles.trim_yaw = param_find("TRIM_YAW");
	_parameter_handles.rollsp_offset_deg = param_find("FW_RSP_OFF");
	_parameter_handles.pitchsp_offset_deg = param_find("FW_PSP_OFF");

	_parameter_handles.man_roll_max = param_find("FW_MAN_R_MAX");
	_parameter_handles.man_pitch_max = param_find("FW_MAN_P_MAX");

	/* fetch initial parameter values */
	parameters_update();
}

FixedWingController::~FixedWingController()
{
	if (_control_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	perf_free(_loop_perf);
	perf_free(_nonfinite_input_perf);
	perf_free(_nonfinite_output_perf);

	att_control::g_control = nullptr;
}

int
FixedWingController::parameters_update()
{
	/* MAGICC PARAMS */
	param_get(_parameter_handles.kp_roll, &(_parameters.kp_roll));
	param_get(_parameter_handles.ki_roll, &(_parameters.ki_roll));
	param_get(_parameter_handles.kd_roll, &(_parameters.kd_roll));
	param_get(_parameter_handles.max_roll_output, &(_parameters.max_aileron_output));
	param_get(_parameter_handles.max_roll_output, &(_parameters.min_aileron_output));
	param_get(_parameter_handles.kp_course, &(_parameters.kp_course));
	param_get(_parameter_handles.ki_course, &(_parameters.ki_course));
	param_get(_parameter_handles.kd_course, &(_parameters.kd_course));
	param_get(_parameter_handles.max_course_output, &(_parameters.max_course_output));
	param_get(_parameter_handles.tau, &(_parameters.tau));

	/* NORIGINAL PARAMS */
	param_get(_parameter_handles.tconst, &(_parameters.tconst));
	param_get(_parameter_handles.p_p, &(_parameters.p_p));
	param_get(_parameter_handles.p_i, &(_parameters.p_i));
	param_get(_parameter_handles.p_ff, &(_parameters.p_ff));
	param_get(_parameter_handles.p_rmax_pos, &(_parameters.p_rmax_pos));
	param_get(_parameter_handles.p_rmax_neg, &(_parameters.p_rmax_neg));
	param_get(_parameter_handles.p_integrator_max, &(_parameters.p_integrator_max));
	param_get(_parameter_handles.p_roll_feedforward, &(_parameters.p_roll_feedforward));

	param_get(_parameter_handles.r_p, &(_parameters.r_p));
	param_get(_parameter_handles.r_i, &(_parameters.r_i));
	param_get(_parameter_handles.r_ff, &(_parameters.r_ff));

	param_get(_parameter_handles.r_integrator_max, &(_parameters.r_integrator_max));
	param_get(_parameter_handles.r_rmax, &(_parameters.r_rmax));

	param_get(_parameter_handles.y_p, &(_parameters.y_p));
	param_get(_parameter_handles.y_i, &(_parameters.y_i));
	param_get(_parameter_handles.y_ff, &(_parameters.y_ff));
	param_get(_parameter_handles.y_integrator_max, &(_parameters.y_integrator_max));
	param_get(_parameter_handles.y_coordinated_min_speed, &(_parameters.y_coordinated_min_speed));
	param_get(_parameter_handles.y_rmax, &(_parameters.y_rmax));

	param_get(_parameter_handles.airspeed_min, &(_parameters.airspeed_min));
	param_get(_parameter_handles.airspeed_trim, &(_parameters.airspeed_trim));
	param_get(_parameter_handles.airspeed_max, &(_parameters.airspeed_max));

	param_get(_parameter_handles.trim_roll, &(_parameters.trim_roll));
	param_get(_parameter_handles.trim_pitch, &(_parameters.trim_pitch));
	param_get(_parameter_handles.trim_yaw, &(_parameters.trim_yaw));
	param_get(_parameter_handles.rollsp_offset_deg, &(_parameters.rollsp_offset_deg));
	param_get(_parameter_handles.pitchsp_offset_deg, &(_parameters.pitchsp_offset_deg));
	_parameters.rollsp_offset_rad = math::radians(_parameters.rollsp_offset_deg);
	_parameters.pitchsp_offset_rad = math::radians(_parameters.pitchsp_offset_deg);
	param_get(_parameter_handles.man_roll_max, &(_parameters.man_roll_max));
	param_get(_parameter_handles.man_pitch_max, &(_parameters.man_pitch_max));
	_parameters.man_roll_max = math::radians(_parameters.man_roll_max);
	_parameters.man_pitch_max = math::radians(_parameters.man_pitch_max);

	/* ORIGINAL CONTROLLERS */
	/* pitch control parameters */
	_pitch_ctrl.set_time_constant(_parameters.tconst);
	_pitch_ctrl.set_k_p(_parameters.p_p);
	_pitch_ctrl.set_k_i(_parameters.p_i);
	_pitch_ctrl.set_k_ff(_parameters.p_ff);
	_pitch_ctrl.set_integrator_max(_parameters.p_integrator_max);
	_pitch_ctrl.set_max_rate_pos(math::radians(_parameters.p_rmax_pos));
	_pitch_ctrl.set_max_rate_neg(math::radians(_parameters.p_rmax_neg));
	_pitch_ctrl.set_roll_ff(_parameters.p_roll_feedforward);

	/* roll control parameters */
	_roll_ctrl.set_time_constant(_parameters.tconst);
	_roll_ctrl.set_k_p(_parameters.r_p);
	_roll_ctrl.set_k_i(_parameters.r_i);
	_roll_ctrl.set_k_ff(_parameters.r_ff);
	_roll_ctrl.set_integrator_max(_parameters.r_integrator_max);
	_roll_ctrl.set_max_rate(math::radians(_parameters.r_rmax));

	/* yaw control parameters */
	_yaw_ctrl.set_k_p(_parameters.y_p);
	_yaw_ctrl.set_k_i(_parameters.y_i);
	_yaw_ctrl.set_k_ff(_parameters.y_ff);
	_yaw_ctrl.set_integrator_max(_parameters.y_integrator_max);
	_yaw_ctrl.set_coordinated_min_speed(_parameters.y_coordinated_min_speed);
	_yaw_ctrl.set_max_rate(math::radians(_parameters.y_rmax));

	return OK;

	/* MAGICC CONTROLLERS */
	_roll_ctrl_MAGICC.setGains(_parameters.kp_roll, _parameters.ki_roll, _parameters.kd_roll);
	_roll_ctrl_MAGICC.setOutputLimits(_parameters.max_roll_output,_parameters.min_roll_ouptut);
	_course_ctrl_MAGICC.setGains(_parameters.kp_course, _parameters.ki_course, _parameters.kd_course);
	_course_ctrl_MAGICC.setOutputLimits(_parameters.max_course_output,_parameters.min_course_ouptut);
}


void
FixedWingController::task_main_trampoline(int argc, char *argv[])
{
	att_control::g_control->task_main();
}

void
FixedWingController::task_main()
{

	/* inform about start */
	warnx("Initializing..");
	fflush(stdout);

	/*
	 * do subscriptions
	 */
	_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_accel_sub = orb_subscribe(ORB_ID(sensor_accel0));
	_airspeed_sub = orb_subscribe(ORB_ID(airspeed));
	_vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_x_hat_sub = orb_subscribe(ORB_ID(x_hat));

	/* rate limit vehicle status updates to 5Hz */
	orb_set_interval(_vcontrol_mode_sub, 200);
	/* rate limit attitude control to 50 Hz (with some margin, so 17 ms) */
	orb_set_interval(_att_sub, 17);

	/* get an initial update for all sensor and status data */
	vehicle_airspeed_poll();
	vehicle_setpoint_poll();
	vehicle_accel_poll();
	vehicle_control_mode_poll();
	vehicle_manual_poll();
	vehicle_status_poll();
	x_command_poll();
	x_hat_poll();

	/* wakeup source(s) */
	struct pollfd fds[2];

	//Declare the setpoints to be used in the PID controllers
	float roll_sp;
	float pitch_sp;
	float throttle_sp;
	float airspeed_sp;	
	float airspeed_scaling;


	//Initialize Pitch-Hold Controller
	float pitch_ki = _parameters.p_i;
	float pitch_kp = _parameters.p_p;
	float pitch_kd = 0;
	float ts = 0.01;
	float pitch_tau = _parameters.tconst;
	float evelator_lim = 45.0 * (3.14159)/180.0
	_pitchHold_ctrl = new UAVpid.UAVpid(&_x_hat.theta, &_delta_e, &_x_command.theta,
		pitch_kp, pitch_ki, pitch_kd, elevator_lim, -elevator_lim,
		ts, pitch_tau);


	//Initialize Altitude-Hold Controller
	float alt_ki;
	float alt_kp;
	float alt_kd = 0;
	float alt_tau;
	float theta_lim = 30.0 * (3.14159)/180.0;
	_altitudeHold_ctrl = new UAVpid.UAVpid(&_x_hat.h, &_x_command.theta, &_x_command.h,
		alt_kp, alt_ki, alt_kd, theta_lim, -theta_lim,
		ts, alt_tau);


	//Initialize Airspeed-With-Pitch-Hold Controller
	float ASP_ki;
	float ASP_kp;
	float ASP_kd = 0;
	float ASP_tau;
	_airspeedPitchHold_ctrl = new UAVpid.UAVpid(&_x_hat.Va, &_x_command.theta, &_x_command.Va,
		ASP_kp, ASP_ki, ASP_kd, theta_lim, -theta_lim,
		ts, ASP_tau);


	//Initialize Airspeed-With-Throttle-Hold Controller
	float AST_ki;
	float AST_kp;
	float AST_kd = 0;
	float AST_tau;
	_airspeedThrottleHold_ctrl = new UAVpid.UAVpid(&_x_hat.Va, &_delta_t, &_x_command.Va,
		AST_kp, AST_ki, AST_kd, 1, 0,
		ts, AST_tau);

	//Initialize Roll Controller
	_rollControl = new UAVpid.UAVpid(&_x_hat.phi, &_delta_a, &_x_command.phi,
					 _parameters.kp_roll, _parameters.ki_roll, _parameters.kd_roll,
					 _parameters.tau,     _parameters.max_roll_output,_parameters.min_roll_ouptut);

	// Initialize Course-Hold Controller
	_courseControl = new UAVpid.UAVpid(&_x_hat.chi, &_x_command.phi, &_x_command.chi,
				           _parameters.kp_course, _parameters.ki_course, _parameters.kd_course,
					   _parameters.tau,_parameters.max_course_output,-1.0*_parameters.max_course_ouptut);

	/* Update Parameters - this function also updates
	 * the gains for the controllers, so it has to happen
	 * after the controllers are initialized
	 */
	parameters_update();

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _att_sub;
	fds[1].events = POLLIN;

	_task_running = true;

	while (!_task_should_exit) {

		static int loop_counter = 0;

		/* wait for up to 500ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);
		/* timed out - periodic check for _task_should_exit, etc. */
		if (pret == 0)
			continue;
		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}
		perf_begin(_loop_perf);

		/* only update parameters if they changed */
		if (fds[0].revents & POLLIN) {
			/* read from param to clear updated flag */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);
			/* update parameters from storage */
			parameters_update();
		}

		/* only run controller if attitude changed */
		if (fds[1].revents & POLLIN) {

			static uint64_t last_run = 0;
			float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too large deltaT's */
			if (deltaT > 1.0f)
				deltaT = 0.01f;

			/* load local copies */
			orb_copy(ORB_ID(vehicle_attitude), _att_sub, &_att);
			vehicle_airspeed_poll();
			vehicle_setpoint_poll();
			vehicle_accel_poll();
			vehicle_control_mode_poll();
			vehicle_manual_poll();
			global_pos_poll();
			vehicle_status_poll();
			x_command_poll();			
			x_hat_poll();

			/* lock integrator until control is started */
			bool lock_integrator;
			if (_vcontrol_mode.flag_control_attitude_enabled) {
				lock_integrator = false;
			} else {
				lock_integrator = true;
			}

			/* Simple handling of failsafe: deploy parachute if failsafe is on */
			if (_vcontrol_mode.flag_control_termination_enabled) {
				_actuators_airframe.control[1] = 1.0f;
//				warnx("_actuators_airframe.control[1] = 1.0f;");
			} else {
				_actuators_airframe.control[1] = 0.0f;
//				warnx("_actuators_airframe.control[1] = -1.0f;");
			}

			/* decide if in stabilized or full manual control */
			if (_vcontrol_mode.flag_control_attitude_enabled) {
			
				compute_longitudinal_control();
				compute_lateral_control();
				/*
				 * For scaling our actuators using anything less than the min (close to stall)
				 * speed doesn't make any sense - its the strongest reasonable deflection we
				 * want to do in flight and its the baseline a human pilot would choose.
				 *
				 * Forcing the scaling to this value allows reasonable handheld tests.
				 */

				airspeed_scaling = _parameters.airspeed_trim / ((airspeed < _parameters.airspeed_min) ? _parameters.airspeed_min : airspeed);

				//reset the set points
				roll_sp = _parameters.rollsp_offset_rad;
				pitch_sp = _parameters.pitchsp_offset_rad;
				throttle_sp = 0.0f;

				
				/* Read attitude setpoint from uorb if
				 * - velocity control or position control is enabled (pos controller is running)
				 * - manual control is disabled (another app may send the setpoint, but it should
				 *   for sure not be set from the remote control values)
				 */
				if (_vcontrol_mode.flag_control_velocity_enabled ||
						_vcontrol_mode.flag_control_position_enabled ||
						!_vcontrol_mode.flag_control_manual_enabled) {
					/* read in attitude setpoint from attitude setpoint uorb topic */
					roll_sp = _att_sp.roll_body + _parameters.rollsp_offset_rad;
					pitch_sp = _att_sp.pitch_body + _parameters.pitchsp_offset_rad;
					throttle_sp = _att_sp.thrust;

					/* reset integrals where needed */
					if (_att_sp.roll_reset_integral) {
						_roll_ctrl.reset_integrator();
					}
					if (_att_sp.pitch_reset_integral) {
						_pitch_ctrl.reset_integrator();
					}
					if (_att_sp.yaw_reset_integral) {
						_yaw_ctrl.reset_integrator();
					}
				} else {
					/*
					 * Scale down roll and pitch as the setpoints are radians
					 * and a typical remote can only do around 45 degrees, the mapping is
					 * -1..+1 to -man_roll_max rad..+man_roll_max rad (equivalent for pitch)
					 *
					 * With this mapping the stick angle is a 1:1 representation of
					 * the commanded attitude.
					 *
					 * The trim gets subtracted here from the manual setpoint to get
					 * the intended attitude setpoint. Later, after the rate control step the
					 * trim is added again to the control signal.
					 */
					roll_sp = (_manual.y * _parameters.man_roll_max - _parameters.trim_roll)
						+ _parameters.rollsp_offset_rad;
					pitch_sp = -(_manual.x * _parameters.man_pitch_max - _parameters.trim_pitch)
						+ _parameters.pitchsp_offset_rad;
					throttle_sp = _manual.z;
					_actuators.control[4] = _manual.flaps;

					/*
					 * in manual mode no external source should / does emit attitude setpoints.
					 * emit the manual setpoint here to allow attitude controller tuning
					 * in attitude control mode.
					 */
					struct vehicle_attitude_setpoint_s att_sp;
					att_sp.timestamp = hrt_absolute_time();
					att_sp.roll_body = roll_sp;
					att_sp.pitch_body = pitch_sp;
					att_sp.yaw_body = 0.0f - _parameters.trim_yaw;
					att_sp.thrust = throttle_sp;

					/* lazily publish the setpoint only once available */
					if (_attitude_sp_pub > 0) {
						/* publish the attitude setpoint */
						orb_publish(ORB_ID(vehicle_attitude_setpoint), _attitude_sp_pub, &att_sp);

					} else {
						/* advertise and publish */
						_attitude_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);
					}
				}

				/* If the aircraft is on ground reset the integrators */
				if (_vehicle_status.condition_landed) {
					_roll_ctrl.reset_integrator();
					_pitch_ctrl.reset_integrator();
					_yaw_ctrl.reset_integrator();
				}

				/* Prepare speed_body_u and speed_body_w */
				float speed_body_u = 0.0f;
				float speed_body_v = 0.0f;
				float speed_body_w = 0.0f;
				if(_att.R_valid) 	{
					speed_body_u = _att.R[0][0] * _global_pos.vel_n + _att.R[1][0] * _global_pos.vel_e + _att.R[2][0] * _global_pos.vel_d;
					speed_body_v = _att.R[0][1] * _global_pos.vel_n + _att.R[1][1] * _global_pos.vel_e + _att.R[2][1] * _global_pos.vel_d;
					speed_body_w = _att.R[0][2] * _global_pos.vel_n + _att.R[1][2] * _global_pos.vel_e + _att.R[2][2] * _global_pos.vel_d;
				} else	{
					if (_debug && loop_counter % 10 == 0) {
						warnx("Did not get a valid R\n");
					}
				}

				/* Run Longitudinal Controller */
				float h = _x_hat.h;
				float h_c = -_x_command.pd;

				if(h < _parameters.altitude_takeoff_zone)
				{
					//In take-off zone
					_delta_t = 1;
					_x_command.theta = _parameters.theta_takeoff;
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

				_actuators.control[0] = _delta_e;
				_actuators.control[1] = _delta_a;
				_actuators.control[2] = _delta_r;
				_actuators.control[3] = _delta_t;



				if (isfinite(roll_sp) && isfinite(pitch_sp)) {
					_roll_ctrl.control_attitude(roll_sp, _att.roll);
					_pitch_ctrl.control_attitude(pitch_sp, _att.roll, _att.pitch, airspeed);
					_yaw_ctrl.control_attitude(_att.roll, _att.pitch,
							speed_body_u, speed_body_v, speed_body_w,
							_roll_ctrl.get_desired_rate(), _pitch_ctrl.get_desired_rate()); //runs last, because is depending on output of roll and pitch attitude

					/* Run attitude RATE controllers which need the desired attitudes from above, add trim */
					float roll_u = _roll_ctrl.control_bodyrate(_att.pitch,
							_att.rollspeed, _att.yawspeed,
							_yaw_ctrl.get_desired_rate(),
							_parameters.airspeed_min, _parameters.airspeed_max, airspeed, airspeed_scaling, lock_integrator);
					_actuators.control[0] = (isfinite(roll_u)) ? roll_u + _parameters.trim_roll : _parameters.trim_roll;
					if (!isfinite(roll_u)) {
						_roll_ctrl.reset_integrator();
						perf_count(_nonfinite_output_perf);

						if (_debug && loop_counter % 10 == 0) {
							warnx("roll_u %.4f", (double)roll_u);
						}
					}

					float pitch_u = _pitch_ctrl.control_bodyrate(_att.roll, _att.pitch,
							_att.pitchspeed, _att.yawspeed,
							_yaw_ctrl.get_desired_rate(),
							_parameters.airspeed_min, _parameters.airspeed_max, airspeed, airspeed_scaling, lock_integrator);
					_actuators.control[1] = (isfinite(pitch_u)) ? pitch_u + _parameters.trim_pitch : _parameters.trim_pitch;
					if (!isfinite(pitch_u)) {
						_pitch_ctrl.reset_integrator();
						perf_count(_nonfinite_output_perf);
						if (_debug && loop_counter % 10 == 0) {
							warnx("pitch_u %.4f, _yaw_ctrl.get_desired_rate() %.4f,"
								" airspeed %.4f, airspeed_scaling %.4f,"
								" roll_sp %.4f, pitch_sp %.4f,"
								" _roll_ctrl.get_desired_rate() %.4f,"
								" _pitch_ctrl.get_desired_rate() %.4f"
								" att_sp.roll_body %.4f",
								(double)pitch_u, (double)_yaw_ctrl.get_desired_rate(),
								(double)airspeed, (double)airspeed_scaling,
								(double)roll_sp, (double)pitch_sp,
								(double)_roll_ctrl.get_desired_rate(),
								(double)_pitch_ctrl.get_desired_rate(),
								(double)_att_sp.roll_body);
						}
					}

					float yaw_u = _yaw_ctrl.control_bodyrate(_att.roll, _att.pitch,
							_att.pitchspeed, _att.yawspeed,
							_pitch_ctrl.get_desired_rate(),
							_parameters.airspeed_min, _parameters.airspeed_max, airspeed, airspeed_scaling, lock_integrator);
					_actuators.control[2] = (isfinite(yaw_u)) ? yaw_u + _parameters.trim_yaw : _parameters.trim_yaw;
					if (!isfinite(yaw_u)) {
						_yaw_ctrl.reset_integrator();
						perf_count(_nonfinite_output_perf);
						if (_debug && loop_counter % 10 == 0) {
							warnx("yaw_u %.4f", (double)yaw_u);
						}
					}

					/* throttle passed through if it is finite and if no engine failure was
					 * detected */
					_actuators.control[3] = (isfinite(throttle_sp) &&
							!(_vehicle_status.engine_failure ||
								_vehicle_status.engine_failure_cmd)) ?
						throttle_sp : 0.0f;
					if (!isfinite(throttle_sp)) {
						if (_debug && loop_counter % 10 == 0) {
							warnx("throttle_sp %.4f", (double)throttle_sp);
						}
					}
				} else {
					perf_count(_nonfinite_input_perf);
					if (_debug && loop_counter % 10 == 0) {
						warnx("Non-finite setpoint roll_sp: %.4f, pitch_sp %.4f", (double)roll_sp, (double)pitch_sp);
					}
				}

				/*
				 * Lazily publish the rate setpoint (for analysis, the actuators are published below)
				 * only once available
				 */
				vehicle_rates_setpoint_s rates_sp;
				rates_sp.roll = _roll_ctrl.get_desired_rate();
				rates_sp.pitch = _pitch_ctrl.get_desired_rate();
				rates_sp.yaw = _yaw_ctrl.get_desired_rate();

				rates_sp.timestamp = hrt_absolute_time();

				if (_rate_sp_pub > 0) {
					/* publish the attitude setpoint */
					orb_publish(ORB_ID(vehicle_rates_setpoint), _rate_sp_pub, &rates_sp);

				} else {
					/* advertise and publish */
					_rate_sp_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &rates_sp);
				}

			} else {
				/* manual/direct control */
				_actuators.control[0] = _manual.y;
				_actuators.control[1] = -_manual.x;
				_actuators.control[2] = _manual.r;
				_actuators.control[3] = _manual.z;
				_actuators.control[4] = _manual.flaps;
			}

			_actuators.control[5] = _manual.aux1;
			_actuators.control[6] = _manual.aux2;
			_actuators.control[7] = _manual.aux3;

			/* lazily publish the setpoint only once available */
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

		loop_counter++;
		perf_end(_loop_perf);
	}

	warnx("exiting.\n");

	_control_task = -1;
	_task_running = false;
	_exit(0);
}

int
FixedWingController::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = task_spawn_cmd("fw_att_control",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_MAX - 5,
				       2048,
				       (main_t)&FixedWingController::task_main_trampoline,
				       nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int fw_att_control_main(int argc, char *argv[])
{
	if (argc < 1)
		errx(1, "usage: fw_att_control {start|stop|status}");

	if (!strcmp(argv[1], "start")) {

		if (att_control::g_control != nullptr)
			errx(1, "already running");

		att_control::g_control = new FixedWingController;

		if (att_control::g_control == nullptr)
			errx(1, "alloc failed");

		if (OK != att_control::g_control->start()) {
			delete att_control::g_control;
			att_control::g_control = nullptr;
			err(1, "start failed");
		}

		/* avoid memory fragmentation by not exiting start handler until the task has fully started */
		while (att_control::g_control == nullptr || !att_control::g_control->task_running()) {
			usleep(50000);
			printf(".");
			fflush(stdout);
		}
		printf("\n");

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (att_control::g_control == nullptr)
			errx(1, "not running");

		delete att_control::g_control;
		att_control::g_control = nullptr;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (att_control::g_control) {
			errx(0, "running");

		} else {
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	return 1;
}

} // end namespace