/****************************************************************************
 *
 *   Copyright (c) 2013 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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

/****************************************************************************
*  Header file for the fixed wing position control, developed in ECEn 674
*  at Brigham Young University.
*
****************************************************************************/

#pragma once 

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/navigation_capabilities.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/parameter_update.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/pid/pid.h>
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>
#include <mavlink/mavlink_log.h>
#include <launchdetection/LaunchDetector.h>
#include <ecl/l1/ecl_l1_pos_controller.h>
#include <external_lgpl/tecs/tecs.h>
#include "landingslope.h"
#include <systemlib/mtecs/mTecs.h>

using namespace launchdetection;


class FixedwingPositionControl
{
public:
	/**
	 * Constructor
	 */
	FixedwingPositionControl();

	/**
	 * Destructor, also kills the sensors task.
	 */
	~FixedwingPositionControl();

	/**
	 * Start the sensors task.
	 *
	 * @return	OK on success.
	 */
	static int	start();

	/**
	 * Task status
	 *
	 * @return	true if the mainloop is running
	 */
	bool		task_running() { return _task_running; }

private:
	int		_mavlink_fd;

	bool		_task_should_exit;		/**< if true, sensor task should exit */
	bool		_task_running;			/**< if true, task is running in its mainloop */

	int		_global_pos_sub;
	int		_pos_sp_triplet_sub;
	int		_att_sub;			/**< vehicle attitude subscription */
	int		_airspeed_sub;			/**< airspeed subscription */
	int		_control_mode_sub;		/**< control mode subscription */
	int		_vehicle_status_sub;		/**< vehicle status subscription */
	int 		_params_sub;			/**< notification of parameter updates */
	int 		_manual_control_sub;		/**< notification of manual control updates */
	int		_sensor_combined_sub;		/**< for body frame accelerations */

	orb_advert_t	_attitude_sp_pub;		/**< attitude setpoint */
	orb_advert_t	_tecs_status_pub;		/**< TECS status publication */
	orb_advert_t	_nav_capabilities_pub;		/**< navigation capabilities publication */

	struct vehicle_attitude_s			_att;				/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s		_att_sp;			/**< vehicle attitude setpoint */
	struct navigation_capabilities_s		_nav_capabilities;		/**< navigation capabilities */
	struct manual_control_setpoint_s		_manual;			/**< r/c channel data */
	struct airspeed_s				_airspeed;			/**< airspeed */
	struct vehicle_control_mode_s			_control_mode;			/**< control mode */
	struct vehicle_status_s				_vehicle_status;		/**< vehicle status */
	struct vehicle_global_position_s		_global_pos;			/**< global vehicle position */
	struct position_setpoint_triplet_s		_pos_sp_triplet;		/**< triplet of mission items */
	struct sensor_combined_s			_sensor_combined;		/**< for body frame accelerations */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	/* land states */
	bool land_noreturn_horizontal;
	bool land_noreturn_vertical;
	bool land_stayonground;
	bool land_motor_lim;
	bool land_onslope;
	bool land_useterrain;

	/* takeoff/launch states */
	LaunchDetectionResult launch_detection_state;

	bool last_manual;				///< true if the last iteration was in manual mode (used to determine when a reset is needed)

	/* Landingslope object */
	Landingslope landingslope;
	float flare_curve_alt_rel_last;

	/* heading hold */
	float target_bearing;

	/* Launch detection */
	launchdetection::LaunchDetector launchDetector;

	/* throttle and airspeed states */
	float _airspeed_error;				///< airspeed error to setpoint in m/s
	bool _airspeed_valid;				///< flag if a valid airspeed estimate exists
	uint64_t _airspeed_last_valid;			///< last time airspeed was valid. Used to detect sensor failures
	float _groundspeed_undershoot;			///< ground speed error to min. speed in m/s
	bool _global_pos_valid;				///< global position is valid
	math::Matrix<3, 3> _R_nb;			///< current attitude

	ECL_L1_Pos_Controller		_l1_control;
	TECS						_tecs;
	fwPosctrl::mTecs				_mTecs;
	bool						_was_pos_control_mode;

	struct {
		float l1_period;
		float l1_damping;

		float time_const;
		float time_const_throt;
		float min_sink_rate;
		float max_sink_rate;
		float max_climb_rate;
		float climbout_diff;
		float heightrate_p;
		float heightrate_ff;
		float speedrate_p;
		float throttle_damp;
		float integrator_gain;
		float vertical_accel_limit;
		float height_comp_filter_omega;
		float speed_comp_filter_omega;
		float roll_throttle_compensation;
		float speed_weight;
		float pitch_damping;

		float airspeed_min;
		float airspeed_trim;
		float airspeed_max;

		float pitch_limit_min;
		float pitch_limit_max;
		float roll_limit;
		float throttle_min;
		float throttle_max;
		float throttle_cruise;
		float throttle_slew_max;

		float throttle_land_max;

		float land_slope_angle;
		float land_H1_virt;
		float land_flare_alt_relative;
		float land_thrust_lim_alt_relative;
		float land_heading_hold_horizontal_distance;
		float land_flare_pitch_min_deg;
		float land_flare_pitch_max_deg;
		int land_use_terrain_estimate;

		float k_orbit;

	}		_parameters;			/**< local copies of interesting parameters */

	struct {

		param_t l1_period;
		param_t l1_damping;

		param_t time_const;
		param_t time_const_throt;
		param_t min_sink_rate;
		param_t max_sink_rate;
		param_t max_climb_rate;
		param_t climbout_diff;
		param_t heightrate_p;
		param_t heightrate_ff;
		param_t speedrate_p;
		param_t throttle_damp;
		param_t integrator_gain;
		param_t vertical_accel_limit;
		param_t height_comp_filter_omega;
		param_t speed_comp_filter_omega;
		param_t roll_throttle_compensation;
		param_t speed_weight;
		param_t pitch_damping;

		param_t airspeed_min;
		param_t airspeed_trim;
		param_t airspeed_max;

		param_t pitch_limit_min;
		param_t pitch_limit_max;
		param_t roll_limit;
		param_t throttle_min;
		param_t throttle_max;
		param_t throttle_cruise;
		param_t throttle_slew_max;

		param_t throttle_land_max;

		param_t land_slope_angle;
		param_t land_H1_virt;
		param_t land_flare_alt_relative;
		param_t land_thrust_lim_alt_relative;
		param_t land_heading_hold_horizontal_distance;
		param_t land_flare_pitch_min_deg;
		param_t land_flare_pitch_max_deg;
		param_t land_use_terrain_estimate;

		param_t orbit_k_orbit;

	}		_parameter_handles;		/**< handles for interesting parameters */


	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	/**
	 * Update control outputs
	 *
	 */
	void		control_update();

	/**
	 * Check for changes in control mode
	 */
	void		vehicle_control_mode_poll();

	/**
	 * Check for changes in vehicle status.
	 */
	void		vehicle_status_poll();

	/**
	 * Check for airspeed updates.
	 */
	bool		vehicle_airspeed_poll();

	/**
	 * Check for position updates.
	 */
	void		vehicle_attitude_poll();

	/**
	 * Check for accel updates.
	 */
	void		vehicle_sensor_combined_poll();

	/**
	 * Check for set triplet updates.
	 */
	void		vehicle_setpoint_poll();

	/**
	 * Publish navigation capabilities
	 */
	void navigation_capabilities_publish();

	/**
	 * Return the terrain estimate during landing: uses the wp altitude value or the terrain estimate if available
	 */
	float get_terrain_altitude_landing(float land_setpoint_alt, const struct vehicle_global_position_s &global_pos);

	/**
	 * Control position.
	 */
	bool		control_position(const math::Vector<2> &global_pos, const math::Vector<3> &ground_speed,
					 const struct position_setpoint_triplet_s &_pos_sp_triplet);

	float calculate_target_airspeed(float airspeed_demand);
	void calculate_gndspeed_undershoot(const math::Vector<2> &current_position, const math::Vector<2> &ground_speed_2d, const struct position_setpoint_triplet_s &pos_sp_triplet);

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main sensor collection task.
	 */
	void		task_main();

	/*
	 * Reset takeoff state
	 */
	void reset_takeoff_state();

	/*
	 * Reset landing state
	 */
	void reset_landing_state();


	/*
	 * Call TECS : a wrapper function to call one of the TECS implementations (mTECS is called only if enabled via parameter)
	 * XXX need to clean up/remove this function once mtecs fully replaces TECS
	 */
	void tecs_update_pitch_throttle(float alt_sp, float v_sp, float eas2tas,
			float pitch_min_rad, float pitch_max_rad,
			float throttle_min, float throttle_max, float throttle_cruise,
			bool climbout_mode, float climbout_pitch_min_rad,
			float altitude,
			const math::Vector<3> &ground_speed,
			tecs_mode mode = TECS_MODE_NORMAL,
			bool pitch_max_special = false);

	void navigate_loiter(const math::Vector<2> &curr_wp, const math::Vector<2> &current_position, float rhoOrbit, int8_t lambda,
				       const math::Vector<2> &ground_speed_vector);

	void navigate_waypoints(const math::Vector<2> &prev_wp, const math::Vector<2> &curr_wp, const math::Vector<2> &current_position,
				       const math::Vector<2> &ground_speed_vector);

};

namespace l1_control
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

extern FixedwingPositionControl	*g_control ;//= nullptr;
}

using namespace l1_control;


/*
void FixedwingPositionControl::vehicle_control_mode_poll();

void FixedwingPositionControl::vehicle_status_poll();

bool FixedwingPositionControl::vehicle_airspeed_poll();

void FixedwingPositionControl::vehicle_attitude_poll();

void FixedwingPositionControl::vehicle_sensor_combined_poll();

void FixedwingPositionControl::vehicle_setpoint_poll();*/