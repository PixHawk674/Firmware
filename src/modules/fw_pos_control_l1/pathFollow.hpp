// pathFollow.hpp 

#include "fw_pos_control_l1.hpp"
#include <math.h>
#include <cmath>

void FixedwingPositionControl::navigate_loiter(const math::Vector<2> &curr_wp, const math::Vector<2> &current_position, float rhoOrbit, int8_t lambda,
				       const math::Vector<2> &ground_speed_vector); +