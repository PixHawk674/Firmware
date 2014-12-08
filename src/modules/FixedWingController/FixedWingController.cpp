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

#include "FixedWingController.h"
#include "FixedWingControllerDaemon.h"

namespace FixedWingController
{

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
	_loop_perf(perf_alloc(PC_ELAPSED, "FixedWingController")),
	_nonfinite_input_perf(perf_alloc(PC_COUNT, "fFixedWingController nonfinite input")),
	_nonfinite_output_perf(perf_alloc(PC_COUNT, "FixedWingController nonfinite output")),
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
	_parameter_handles.max_aileron_output = param_find("FW_MAX_AILERON_OUTPUT");
	_parameter_handles.min_aileron_output = param_find("FW_MIN_AILERON_OUTPUT");
	_parameter_handles.kp_course = param_find("FW_COURSE_P");
	_parameter_handles.ki_course = param_find("FW_COURSE_I");
	_parameter_handles.kd_course = param_find("FW_COURSE_D");
	_parameter_handles.max_course_output = param_find("FW_MAX_COURSE_OUTPUT");
	_parameter_handles.tau = param_find("FW_TAU");
	_parameter_handles.ts = param_find("FW_TS");

	_parameter_handles.ts = param_find("FW_PITCH_P");
	_parameter_handles.ts = param_find("FW_PITCH_I");
	_parameter_handles.ts = param_find("FW_PITCH_D");
	_parameter_handles.ts = param_find("FW_ALT_P");
	_parameter_handles.ts = param_find("FW_ALT_I");
	_parameter_handles.ts = param_find("FW_ALT_D");
	_parameter_handles.ts = param_find("FW_VA_BY_PITCH_P");
	_parameter_handles.ts = param_find("FW_VA_BY_PITCH_I");
	_parameter_handles.ts = param_find("FW_VA_BY_PITCH_D");
	_parameter_handles.ts = param_find("FW_VA_BY_THROTTLE_P");
	_parameter_handles.ts = param_find("FW_VA_BY_THROTTLE_I");
	_parameter_handles.ts = param_find("FW_VA_BY_THROTTLE_D");
	_parameter_handles.ts = param_find("FW_MAX_ELEVATOR_OUTPUT");
	_parameter_handles.ts = param_find("FW_MIN_ELEVATOR_OUTPUT");

	_parameter_handles.altitude_takeoff_zone = param_find("ALT_TAKE_OFF_ZONE");
	_parameter_handles.altitude_hold_zone = param_find("ALT_HOLD_ZONE");
	_parameter_handles.theta_takeoff = param_find("THETA_TAKE_OFF");

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

	FixedWingControllerDaemon::g_control = nullptr;
}

int
FixedWingController::parameters_update()
{
	/* MAGICC PARAMS */
	param_get(_parameter_handles.kp_roll, &(_parameters.kp_roll));
	param_get(_parameter_handles.ki_roll, &(_parameters.ki_roll));
	param_get(_parameter_handles.kd_roll, &(_parameters.kd_roll));
	param_get(_parameter_handles.max_aileron_output, &(_parameters.max_aileron_output));
	param_get(_parameter_handles.max_aileron_output, &(_parameters.min_aileron_output));
	param_get(_parameter_handles.kp_course, &(_parameters.kp_course));
	param_get(_parameter_handles.ki_course, &(_parameters.ki_course));
	param_get(_parameter_handles.kd_course, &(_parameters.kd_course));
	param_get(_parameter_handles.max_course_output, &(_parameters.max_course_output));

	param_get(_parameter_handles.kp_pitch, &(_parameters.kp_pitch));
	param_get(_parameter_handles.ki_pitch, &(_parameters.ki_pitch));
	param_get(_parameter_handles.kd_pitch, &(_parameters.kd_pitch));

	param_get(_parameter_handles.ki_alt, &(_parameters.ki_alt));
	param_get(_parameter_handles.kp_alt, &(_parameters.kp_alt));
	param_get(_parameter_handles.kd_alt, &(_parameters.kd_alt));
	param_get(_parameter_handles.max_elevator_output, &(_parameters.max_elevator_output));
	param_get(_parameter_handles.min_elevator_output, &(_parameters.min_elevator_output));

	param_get(_parameter_handles.ki_VaByPitch, &(_parameters.ki_VaByPitch));
	param_get(_parameter_handles.kp_VaByPitch, &(_parameters.kp_VaByPitch));
	param_get(_parameter_handles.kd_VaByPitch, &(_parameters.kd_VaByPitch));

	param_get(_parameter_handles.ki_VaByThrottle, &(_parameters.ki_VaByThrottle));
	param_get(_parameter_handles.kp_VaByThrottle, &(_parameters.kp_VaByThrottle));
	param_get(_parameter_handles.kd_VaByThrottle, &(_parameters.kd_VaByThrottle));

	param_get(_parameter_handles.tau, &(_parameters.tau));
	param_get(_parameter_handles.ts, &(_parameters.ts));

	param_get(_parameter_handles.altitude_hold_zone, & (_parameters.altitude_hold_zone));
	param_get(_parameter_handles.altitude_takeoff_zone, & (_parameters.altitude_takeoff_zone));
	param_get(_parameter_handles.theta_takeoff, & (_parameters.theta_takeoff));


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

	/* MAGICC CONTROLLERS */
	_rollControl->setGains(_parameters.kp_roll, _parameters.ki_roll, _parameters.kd_roll);
	_rollControl->setOutputLimits(_parameters.max_aileron_output,_parameters.min_aileron_output);
	_courseControl->setGains(_parameters.kp_course, _parameters.ki_course, _parameters.kd_course);
	_courseControl->setOutputLimits(_parameters.max_course_output, -1.0f*_parameters.max_course_output);

	return OK;


}


void
FixedWingController::task_main_trampoline(int argc, char *argv[])
{
	FixedWingControllerDaemon::g_control->task_main();
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

	//Initialize Pitch-Hold Controller
	float pitch_ki = _parameters.ki_pitch;
	float pitch_kp = _parameters.kp_pitch;
	float pitch_kd = _parameters.kd_pitch;
	float ts = _parameters.ts;
	float pitch_tau = _parameters.tau;
	_pitchHold_ctrl = new UAVpid(&_x_hat.theta, &_delta_e, &_x_command.theta,
		pitch_kp, pitch_ki, pitch_kd, ts, pitch_tau,
		_parameters.max_elevator_output, _parameters.min_elevator_output);


	//Initialize Altitude-Hold Controller
	float alt_ki = _parameters.ki_alt;
	float alt_kp = _parameters.kp_alt;
	float alt_kd = _parameters.kd_alt;
	float alt_tau = _parameters.tau;
	float theta_lim = 30.0 * (3.14159)/180.0;
	_altitudeHold_ctrl = new UAVpid(&_x_hat.h, &_x_command.theta, &_x_command.h,
		alt_kp, alt_ki, alt_kd, ts, alt_tau,
		theta_lim, -theta_lim);

	//Initialize Airspeed-With-Pitch-Hold Controller
	float ASP_ki = _parameters.ki_VaByPitch;
	float ASP_kp = _parameters.kp_VaByPitch;
	float ASP_kd = _parameters.kd_VaByPitch;
	float ASP_tau = _parameters.tau;
	_airspeedPitchHold_ctrl = new UAVpid(&_x_hat.Va, &_x_command.theta, &_x_command.Va,
		ASP_kp, ASP_ki, ASP_kd, ts, ASP_tau,
		theta_lim, -theta_lim);


	//Initialize Airspeed-With-Throttle-Hold Controller
	float AST_ki = _parameters.ki_VaByThrottle;
	float AST_kp = _parameters.kp_VaByThrottle;
	float AST_kd = _parameters.kd_VaByThrottle;
	float AST_tau = _parameters.tau;
	_airspeedThrottleHold_ctrl = new UAVpid(&_x_hat.Va, &_delta_t, &_x_command.Va,
		AST_kp, AST_ki, AST_kd, ts, AST_tau,
		1, 0);

	//Initialize Roll Controller
	_rollControl = new UAVpid(&_x_hat.phi, &_delta_a, &_x_command.phi,
					 _parameters.kp_roll, _parameters.ki_roll, _parameters.kd_roll, _parameters.ts, 
					 _parameters.tau, _parameters.max_aileron_output,_parameters.min_aileron_output);

	// Initialize Course-Hold Controller
	_courseControl = new UAVpid(&_x_hat.chi, &_x_command.phi, &_x_command.chi,
				           _parameters.kp_course, _parameters.ki_course, _parameters.kd_course, _parameters.ts,
					   _parameters.tau,_parameters.max_course_output,-1.0f*_parameters.max_course_output);

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

			/* lock integrator until control is started 
			bool lock_integrator;
			if (_vcontrol_mode.flag_control_attitude_enabled) {
				lock_integrator = false;
			} else {
				lock_integrator = true;
			}*/

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
				// stabilized mode

				
				/* Convert RC Setpoints to x_command Setpoints if...
				 * - velocity control or position control is not enabled (pos controller is running)
				 * - manual control is enabled (another app may send the setpoint, but it should
				 *   for sure not be set from the remote control values)
				 */
				if (!_vcontrol_mode.flag_control_velocity_enabled ||
						!_vcontrol_mode.flag_control_position_enabled ||
						_vcontrol_mode.flag_control_manual_enabled){

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
					_x_command.phi = (_manual.y * _parameters.man_roll_max - _parameters.trim_roll)
						+ _parameters.rollsp_offset_rad;
					_x_command.theta = -(_manual.x * _parameters.man_pitch_max - _parameters.trim_pitch)
						+ _parameters.pitchsp_offset_rad;
					_delta_t = _manual.z;
					_actuators.control[4] = _manual.flaps;

					/*
					 * in manual mode no external source should / does emit attitude setpoints.
					 * emit the manual setpoint here to allow attitude controller tuning
					 * in attitude control mode.
					 */
					 publish_converted_setpoint();
				}
				else{
					// in full autopilot mode
					compute_control();
					// pass newly computed outputs to actuator controls
					_actuators.control[0] = _delta_a;
					_actuators.control[1] = _delta_e;
					_actuators.control[2] = _delta_r;
					_actuators.control[3] = _delta_t;
				}

			} else {
				/* manual/direct control */
				_actuators.control[0] = _manual.y;
				_actuators.control[1] = -_manual.x;
				_actuators.control[2] = _manual.r;
				_actuators.control[3] = _manual.z;
				_actuators.control[4] = _manual.flaps;
			}

			// Don't touch auxiliary controls
			_actuators.control[5] = _manual.aux1;
			_actuators.control[6] = _manual.aux2;
			_actuators.control[7] = _manual.aux3;

			/* lazily publish the setpoint only once available */
			publish_actuators();
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
	_control_task = task_spawn_cmd("FixedWingController",
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

} // end namespace