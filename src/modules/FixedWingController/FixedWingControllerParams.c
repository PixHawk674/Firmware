/****************************************************************************
 *
f *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
 * @file fw_att_control_params.c
 *
 * Parameters defined by the fixed-wing attitude control task
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#include <nuttx/config.h>

#include <systemlib/param/param.h>


/*
 * Controller parameters, accessible via MAVLink
 *
 */

/********  Roll Controller **********/
/**
 * Roll Controller P Gain
 *
 * Proportional Gain for Roll Controller
 *
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_ROLL_P,1.0f);

/**
 * Roll Controller I Gain
 *
 * Integral Gain for Roll Controller
 *
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_ROLL_I,1.0f);

/**
 * Roll Controller D Gain
 *
 * Derivative Gain for Roll Controller
 *
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_ROLL_D,1.0f);

/**
 * Roll Maximum Actuator Control
 * 
 * Maximum output on aileron output
 */
PARAM_DEFINE_FLOAT(FW_MAX_AILERON_OUTPUT,2000.f);

/**
 * Roll Minimum Actuator Control
 * 
 * Minimum output on aileron output
 */
PARAM_DEFINE_FLOAT(FW_MIN_AILERON_OUTPUT,500.0f);

/********  Course Controller **********/
/**
 * Course Controller P Gain
 *
 * Proportional Gain for Course Controller
 *
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_COURSE_P,1.0f);

/**
 * Course Controller I Gain
 *
 * Integral Gain for Course Controller
 *
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_COURSE_I,1.0f);

/**
 * Course Controller D Gain
 *
 * Derivative Gain for Course Controller
 *
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_COURSE_D,1.0f);

/**
 * Roll Maximum Actuator Control
 * 
 * Maximum output to roll command (deg)
 */
PARAM_DEFINE_FLOAT(FW_MAX_COURSE_OUTPUT,45.0f);

/**
 * Tau
 *
 * Constant for Low-Pass-Filter in Dirty Derivatives of
 * Autopilot PID loops
 */
PARAM_DEFINE_FLOAT(FW_TAU,0.03f);


/**
 * Ts
 *
 * The Sample Rate for the PID loops
 *
 */
PARAM_DEFINE_FLOAT(FW_TS, 0.01f);



