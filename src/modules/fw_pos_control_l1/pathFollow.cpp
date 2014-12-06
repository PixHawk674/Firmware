//navigate_loiter(math::Vector<2> curr_wp, math::Vector<2> current_position, pos_sp_triplet.current.loiter_radius,
//						  pos_sp_triplet.current.loiter_direction, math::Vector<2> ground_speed_2d);

#include "fw_pos_control_l1.hpp"
/*
_l1_control	
void set_ECL_vars_custom(float lateral_accel, float L1_distance, bool circle_mode, float nav_bearing, 
			float bearing_error, float crosstrack_error, float target_bearing)
*/

void FixedwingPositionControl::navigate_loiter(const math::Vector<2> &curr_wp, const math::Vector<2> &current_position, float rhoOrbit, int8_t lambda,
				       const math::Vector<2> &ground_speed_vector){

// lambda: +1 for clockwise, -1 for counterclockwise. 

	//float Va_d = ; // Get the desired velocity and pass it in to the function, for use in commandedRadians. 

	/*** K_ORBIT  */

	float kOrbit = 0.2; // CHANGE THIS TO A PARAM

	float cOrbitN = curr_wp(0);
	float cOrbitE = curr_wp(1);

	float pn = current_position(0);
	float pe = current_position(1);

	float headingN = ground_speed_vector(0);
	float headingE = ground_speed_vector(1);

	float phiOrbit = atan2f(pe - cOrbitE, pn - cOrbitN);
	float chi = atan2f(headingE, headingN);  // Calculated from ground speed, atan(x/y)

	float distFromCenter = sqrtf( pow( pn - cOrbitN, 2)  + pow(pe - cOrbitE, 2) );

	float commandedRadians = phiOrbit + lambda * (M_PI_F / 2 + atanf(kOrbit * (distFromCenter - rhoOrbit) / rhoOrbit) );
	// Takes the current radians around the circle, then adds pi/2 * lambda to fly the plane the desired way. The dist-rho term drives 
	// the plane towards the circle. 


/********************* Required Outputs for the class I copied ****************************************/


	float _bearing_error = commandedRadians - chi; // Angle between requested heading and the current heading. 
	bool _circle_mode = true; 

	/* Hoping that _L1_ratio, _K_L1 are set by the class. */
	float ground_speed = math::max(ground_speed_vector.length() , 0.1f); // Ripped from the ECL; apparently, a 0 ground speed is a problem. 
	float _L1_distance = _l1_control.get_L1_ratio() * ground_speed;
	float lateral_accel_sp_center = _l1_control.get_K_L1() * ground_speed * ground_speed / _L1_distance * sinf(_bearing_error);
	float _lateral_accel = lateral_accel_sp_center;


	/* Also ripped from ECL for compatibility. */
	//math::Vector<2> vector_A_to_airplane = get_local_planar_vector(curr_wp, current_position);
	math::Vector<2> vector_A_to_airplane(math::radians((current_position(0) - curr_wp(0))), 
		  math::radians((current_position(1) - curr_wp(1))*cosf(math::radians(curr_wp(0)))));
	math::Vector<2> vector_A_to_airplane_unit;  // This is a unit vector, given the initialization in a few lines. 

	/* prevent NaN when normalizing */
	if (vector_A_to_airplane.length() > FLT_EPSILON) {  // gt floating point minimum resolution. 
		/* store the normalized vector from waypoint A to current position */
		vector_A_to_airplane_unit = vector_A_to_airplane.normalized();
	} else {
		vector_A_to_airplane_unit = vector_A_to_airplane; // Goes to zero. 
	}
	
	float _nav_bearing = atan2f(-vector_A_to_airplane_unit(1) , -vector_A_to_airplane_unit(0));

	float _target_bearing = get_bearing_to_next_waypoint(current_position(0), current_position(1), curr_wp(0), curr_wp(1));

	float _crosstrack_error = distFromCenter - rhoOrbit; 

	_l1_control.set_ECL_vars_custom(_lateral_accel, _L1_distance, _circle_mode, _nav_bearing, 
			_bearing_error, _crosstrack_error, _target_bearing);
	/*
	*   Outputs:
	*	_lateral_accel: Center of the circle, it looks like.  Keep this calculation, I think. 
	*	_circle_mode = true, since we're doing a circle in loiter mode. 
	*	_bearing_error: Angle between requested and current velocity vector. 
	*	_nav_bearing: Bearing from current position to destination.
	*   _crosstrack_error: How far the plane is from the orbit. 
	*   _target_bearing: Bearing to the center of the circle. 
	*/

}	

