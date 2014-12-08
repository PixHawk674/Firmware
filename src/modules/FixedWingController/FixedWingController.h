#ifndef _FIXEDWINGCONTROLLER_H
#define _FIXEDWINGCONTROLLER_H

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
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/x_command.h>
#include <uORB/topics/x_hat.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/pid/pid.h> // to be removed
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <controllib/UAVpid.h>
#include <mathlib/mathlib.h>

#include <ecl/attitude_fw/ecl_pitch_controller.h>
#include <ecl/attitude_fw/ecl_roll_controller.h>
#include <ecl/attitude_fw/ecl_yaw_controller.h>


namespace FixedWingController
{

class FixedWingController
{
public:
	/**
	 * Constructor
	 */
	FixedWingController();

	/**
	 * Destructor, also kills the sensors task.
	 */
	~FixedWingController();

	/**
	 * Start the sensors task.
	 *
	 * @return	OK on success.
	 */
	int		start();

	/**
	 * Task status
	 *
	 * @return	true if the mainloop is running
	 */
	bool		task_running() { return _task_running; }

private:

	bool		_task_should_exit;		/**< if true, sensor task should exit */
	bool		_task_running;			/**< if true, task is running in its mainloop */
	int		_control_task;			/**< task handle for sensor task */

	int		_att_sub;			/**< vehicle attitude subscription */
	int		_accel_sub;			/**< accelerometer subscription */
	int		_att_sp_sub;			/**< vehicle attitude setpoint */
	int		_attitude_sub;			/**< raw rc channels data subscription */
	int		_airspeed_sub;			/**< airspeed subscription */
	int		_vcontrol_mode_sub;		/**< vehicle status subscription */
	int 		_params_sub;			/**< notification of parameter updates */
	int 		_manual_sub;			/**< notification of manual control updates */
	int		_global_pos_sub;		/**< global position subscription */
	int		_vehicle_status_sub;		/**< vehicle status subscription */
	int		_pos_sp_triplet_sub;		/**< waypoing subscriber (hack to get airspeed)*/
	int  		_x_hat_sub;			/**<subscribe to estimator output */

	orb_advert_t	_rate_sp_pub;			/**< rate setpoint publication */
	orb_advert_t	_attitude_sp_pub;		/**< attitude setpoint point */
	orb_advert_t	_actuators_0_pub;		/**< actuator control group 0 setpoint */
	orb_advert_t	_actuators_1_pub;		/**< actuator control group 1 setpoint (Airframe) */

	struct vehicle_attitude_s			_att;			/**< vehicle attitude */
	struct accel_report				_accel;			/**< body frame accelerations */
	struct vehicle_attitude_setpoint_s		_att_sp;		/**< vehicle attitude setpoint */
	struct manual_control_setpoint_s		_manual;		/**< r/c channel data */
	struct airspeed_s				_airspeed;		/**< airspeed */
	struct vehicle_control_mode_s			_vcontrol_mode;		/**< vehicle control mode */
	struct actuator_controls_s			_actuators;		/**< actuator control inputs */
	struct actuator_controls_s			_actuators_airframe;	/**< actuator control inputs */
	struct vehicle_global_position_s		_global_pos;		/**< global position */
	struct vehicle_status_s				_vehicle_status;	/**< vehicle status */
	struct position_setpoint_triplet_s		_pos_sp_triplet;	/**< waypoints, to be replaced by something more elegant later */
	struct x_command_s				_x_command;		/**< commands structure */
	struct x_hat_s					_x_hat;			/**< estimates */
	

	perf_counter_t	_loop_perf;			/**< loop performance counter */
	perf_counter_t	_nonfinite_input_perf;		/**< performance counter for non finite input */
	perf_counter_t	_nonfinite_output_perf;		/**< performance counter for non finite output */

	bool		_setpoint_valid;		/**< flag if the position control setpoint is valid */
	bool		_debug;				/**< if set to true, print debug output */

	struct {
		float tconst;
		float p_p;
		float p_d;
		float p_i;
		float p_ff;
		float p_rmax_pos;
		float p_rmax_neg;
		float p_integrator_max;
		float p_roll_feedforward;
		float r_p;
		float r_d;
		float r_i;
		float r_ff;
		float r_integrator_max;
		float r_rmax;
		float y_p;
		float y_i;
		float y_d;
		float y_ff;
		float y_roll_feedforward;
		float y_integrator_max;
		float y_coordinated_min_speed;
		float y_rmax;

		float airspeed_min;
		float airspeed_trim;
		float airspeed_max;

		float trim_roll;
		float trim_pitch;
		float trim_yaw;
		float rollsp_offset_deg;			/**< Roll Setpoint Offset in deg */
		float pitchsp_offset_deg;			/**< Pitch Setpoint Offset in deg */
		float rollsp_offset_rad;			/**< Roll Setpoint Offset in rad */
		float pitchsp_offset_rad;			/**< Pitch Setpoint Offset in rad */
		float man_roll_max;						/**< Max Roll in rad */
		float man_pitch_max;					/**< Max Pitch in rad */

		// MAGICC parameters
		float kp_roll;
		float ki_roll;
		float kd_roll;
		float max_aileron_output;
		float min_aileron_output;

		float kp_course;
		float ki_course;
		float kd_course;
		float max_course_output;

		float kp_pitch;
		float ki_pitch;
		float kd_pitch;

		float ki_alt;
		float kp_alt;
		float kd_alt;
		float max_elevator_output;
		float min_elevator_output;

		float ki_VaByPitch;
		float kp_VaByPitch;
		float kd_VaByPitch;

		float ki_VaByThrottle;
		float kp_VaByThrottle;
		float kd_VaByThrottle;

		float tau;
		float ts;

		// State Machine Parameters
		float altitude_takeoff_zone;
		float altitude_hold_zone;
		float theta_takeoff;

	}	_parameters;			/**< local copies of interesting parameters */

	struct {

		param_t tconst;
		param_t p_p;
		param_t p_d;
		param_t p_i;
		param_t p_ff;
		param_t p_rmax_pos;
		param_t p_rmax_neg;
		param_t p_integrator_max;
		param_t p_roll_feedforward;
		param_t r_p;
		param_t r_d;
		param_t r_i;
		param_t r_ff;
		param_t r_integrator_max;
		param_t r_rmax;
		param_t y_p;
		param_t y_i;
		param_t y_d;
		param_t y_ff;
		param_t y_roll_feedforward;
		param_t y_integrator_max;
		param_t y_coordinated_min_speed;
		param_t y_rmax;

		param_t airspeed_min;
		param_t airspeed_trim;
		param_t airspeed_max;

		param_t trim_roll;
		param_t trim_pitch;
		param_t trim_yaw;
		param_t rollsp_offset_deg;
		param_t pitchsp_offset_deg;
		param_t man_roll_max;
		param_t man_pitch_max;

		param_t kp_roll;
		param_t ki_roll;
		param_t kd_roll;
		param_t max_aileron_output;
		param_t min_aileron_output;
		param_t kp_course;
		param_t ki_course;
		param_t kd_course;
		param_t max_course_output;

		param_t kp_pitch;
		param_t ki_pitch;
		param_t kd_pitch;
		
		param_t ki_alt;
		param_t kp_alt;
		param_t kd_alt;
		param_t max_elevator_output;
		param_t min_elevator_output;

		param_t ki_VaByPitch;
		param_t kp_VaByPitch;
		param_t kd_VaByPitch;

		param_t ki_VaByThrottle;
		param_t kp_VaByThrottle;
		param_t kd_VaByThrottle;

		param_t tau;
		param_t ts;

		param_t altitude_hold_zone;
		param_t altitude_takeoff_zone;
		param_t theta_takeoff;
	} _parameter_handles;		/**< handles for interesting parameters */


	//bool _lock_integrator; // a boolean that is controlled by the vehicle control mode

	float _ts;
	float _tau;
	UAVpid* _rollControl;
	UAVpid* _courseControl;
	float _phi_c;
	float _chi_c;
	float _delta_e;
	float _delta_a;
	float _delta_r;
	float _delta_t;

/* To be implmented later
	UAVpid::UAVpid* _sideslip_ctrl;
	float _sideslip_ctrl_input;
	float _sideslip_ctrl_output;
	float _sideslip_c;
	float _sideslip_actuator_pos_limit;
	float _sideslip_actuator_neg_limit;
*/

	UAVpid*		_pitchHold_ctrl;
	UAVpid*		_altitudeHold_ctrl;
	UAVpid*		_airspeedPitchHold_ctrl;
	UAVpid*		_airspeedThrottleHold_ctrl;

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
	 * Check for changes in vehicle control mode.
	 */
	void		vehicle_control_mode_poll();

	/**
	 * Check for changes in manual inputs.
	 */
	void		vehicle_manual_poll();


	/**
	 * Check for airspeed updates.
	 */
	void		vehicle_airspeed_poll();
 

	/**
	 * Check for accel updates.
	 */
	void		vehicle_accel_poll();

	/**
	 * Check for set triplet updates.
	 */
	void		vehicle_setpoint_poll();

	/**
	 * Check for global position updates.
	 */
	void		global_pos_poll();

	/**
	 * Check for vehicle status updates.
	 */
	void		vehicle_status_poll();

	/**
	 * Check for new commands
	 */
	 void 		x_command_poll();

	/**
	 * Check for new estimates
	 */
	 void  		x_hat_poll();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main sensor collection task.
	 */
	void		task_main();

	/**
	 * Publishes Actuator Commands.
	 */
	void		publish_actuators();

	/**
	 * Published a converted setpoint message - this is mainly a transitionary sort of program until we are fully migrated to new code style
	 */
	void		publish_converted_setpoint();

	/**
	 * Computes the control outputs
	 */
	void		compute_control();

};

} // end namespace FixedWingController

#endif
