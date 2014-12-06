#include "fw_pos_control_l1.hpp"

math::Vector<2> get_planar_vector(const math::Vector<2> &origin, const math::Vector<2> &target);

void FixedwingPositionControl::navigate_waypoints(const math::Vector<2> &prev_wp, const math::Vector<2> &curr_wp, const math::Vector<2> &current_position,
				       const math::Vector<2> &ground_speed_vector)
{
	float eta; // Angle from heading to the desired line.
	float xtrack_vel; // Velocity perpendicular to line
	float ltrack_vel; // velocity along line
	float _nav_bearing;
	float _lateral_accel;	
	bool _circle_mode;
	float _bearing_error;

	/* get the direction between the last (visited) and next waypoint */
	float _target_bearing = get_bearing_to_next_waypoint(current_position(0), current_position(1), curr_wp(0), curr_wp(1));

	/* enforce a minimum ground speed of 0.1 m/s to avoid singularities */
	float ground_speed = math::max(ground_speed_vector.length(), 0.1f);

	/* calculate the L1 length required for the desired period */
	float _L1_distance = _l1_control.get_L1_ratio() * ground_speed;

	/* calculate vector from A to B */
	math::Vector<2> vector_forward = get_planar_vector(prev_wp, current_position);

	/*
	 * check if waypoints are on top of each other. If yes,
	 * skip A and directly continue to B
	 */
	if (vector_forward.length() < 1.0e-6f) {
		vector_forward = get_planar_vector(current_position, curr_wp);
	}

	vector_forward.normalize();

	/* calculate the vector from waypoint A to the aircraft */
	math::Vector<2> prev_wp_to_airplane = get_planar_vector(prev_wp, current_position);

	/* calculate crosstrack error (output only) */
	float _crosstrack_error = vector_forward % prev_wp_to_airplane;

	/*
	 * If the current position is in a +-135 degree angle behind waypoint A
	 * and further away from A than the L1 distance, then A becomes the L1 point.
	 * If the aircraft is already between A and B normal L1 logic is applied.
	 */
	float distance_A_to_airplane = prev_wp_to_airplane.length();
	float alongTrackDist = prev_wp_to_airplane * vector_forward;

	/* estimate airplane position WRT to B */
	math::Vector<2> curr_wp_to_P_unit = get_planar_vector(curr_wp, current_position).normalized();
	
	/* calculate angle of airplane position vector relative to line) */

	// XXX this could probably also be based solely on the dot product
	float AB_to_BP_bearing = atan2f(curr_wp_to_P_unit % vector_forward, curr_wp_to_P_unit * vector_forward);

	/* extension from [2], fly directly to A */
	if ( (distance_A_to_airplane > _L1_distance ) && (alongTrackDist / math::max(distance_A_to_airplane , 1.0f) ) < -0.7071f) {

		/* calculate eta to fly to waypoint A */

		/* unit vector from waypoint A to current position */
		math::Vector<2> prev_wp_to_airplane_unit = prev_wp_to_airplane.normalized();
		/* velocity across / orthogonal to line */
		xtrack_vel = ground_speed_vector % (-prev_wp_to_airplane_unit);
		/* velocity along line */
		ltrack_vel = ground_speed_vector * (-prev_wp_to_airplane_unit);
		eta = atan2f(xtrack_vel, ltrack_vel);
		/* bearing from current position to L1 point */
		_nav_bearing = atan2f(-prev_wp_to_airplane_unit(1) , -prev_wp_to_airplane_unit(0));

	/*
	 * If the AB vector and the vector from B to airplane point in the same
	 * direction, we have missed the waypoint. At +- 90 degrees we are just passing it.
	 */
	} else if (fabsf(AB_to_BP_bearing) < math::radians(100.0f)) {
		/*
		 * Extension, fly back to waypoint.
		 * 
		 * This corner case is possible if the system was following
		 * the AB line from waypoint A to waypoint B, then is
		 * switched to manual mode (or otherwise misses the waypoint)
		 * and behind the waypoint continues to follow the AB line.
		 */

		/* calculate eta to fly to waypoint B */
		
		/* velocity across / orthogonal to line */
		xtrack_vel = ground_speed_vector % (-curr_wp_to_P_unit);
		/* velocity along line */
		ltrack_vel = ground_speed_vector * (-curr_wp_to_P_unit);
		eta = atan2f(xtrack_vel, ltrack_vel);
		/* bearing from current position to L1 point */
		_nav_bearing = atan2f(-curr_wp_to_P_unit(1) , -curr_wp_to_P_unit(0));

	} else {

		/* calculate eta to fly along the line between A and B */

		/* velocity across / orthogonal to line */
		xtrack_vel = ground_speed_vector % vector_forward;
		/* velocity along line */
		ltrack_vel = ground_speed_vector * vector_forward;
		/* calculate eta2 (angle of velocity vector relative to line) */
		float eta2 = atan2f(xtrack_vel, ltrack_vel);
		/* calculate eta1 (angle to L1 point) */
		float xtrackErr = prev_wp_to_airplane % vector_forward;
		float sine_eta1 = xtrackErr / math::max(_L1_distance , 0.1f);
		/* limit output to 45 degrees */
		sine_eta1 = math::constrain(sine_eta1, -0.7071f, 0.7071f); //sin(pi/4) = 0.7071
		float eta1 = asinf(sine_eta1);
		eta = eta1 + eta2;
		/* bearing from current position to L1 point */
		_nav_bearing = atan2f(vector_forward(1), vector_forward(0)) + eta1;

	}

	/* limit angle to +-90 degrees */
	eta = math::constrain(eta, (-M_PI_F) / 2.0f, +M_PI_F / 2.0f);
	_lateral_accel = _l1_control.get_K_L1() * ground_speed * ground_speed / _L1_distance * sinf(eta);

	/* flying to waypoints, not circling them */
	_circle_mode = false;

	/* the bearing angle, in NED frame */
	_bearing_error = eta;

	_l1_control.set_ECL_vars_custom(_lateral_accel, _L1_distance, _circle_mode, _nav_bearing, 
			_bearing_error, _crosstrack_error, _target_bearing);
}

math::Vector<2> get_planar_vector(const math::Vector<2> &origin, const math::Vector<2> &target) {

	math::Vector<2> out(math::radians((target(0) - origin(0))), math::radians((target(1) - origin(1))*cosf(math::radians(origin(0)))));
	
	return out * static_cast<float>(CONSTANTS_RADIUS_OF_EARTH);
}