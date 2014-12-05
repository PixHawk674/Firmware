/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file vehicle_global_position.h
 * Definition of the global fused WGS84 position uORB topic.
 *
 * @author James Jackson <james.s.jackson@byu.edu>
 */

#ifndef VEHICLE_GLOBAL_POSITION_T_H_
#define VEHICLE_GLOBAL_POSITION_T_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

/**
 * @addtogroup topics
 * @{
 */

 /**
 * Vehicle State Estimate x_hat
 * 
 * This is the vehicle state as described in the UAV book chapter 8
 * pn, pe, and pd are in reference from the inertial frame
 *
 */
struct x_hat_s {
	uint64_t timestamp;
	float pn;
	float pe;
	float h;
	float Va;
	float alpha;
	float beta;
	float phi;
	float theta;
	float chi;
	float p;
	float q;
	float r;
	float Vg;
	float wn;
	float we;
	float psi;
	double covariance[9][9];
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(vehicle_global_position);

#endif
