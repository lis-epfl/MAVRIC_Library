/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*******************************************************************************
 * \file servos_mix_ADAPTIVE_MORPH_default_config.h
 * 
 * \author MAV'RIC Team
 * \author Simon Pyroth
 *   
 * \brief Default configuration for the servo_mix for the MAVRIC ADAPTIVE_MORPH
 *
 ******************************************************************************/


#ifndef SERVOS_MIX_ADAPTIVE_MORPH_DEFAULT_CONFIG_H_
#define SERVOS_MIX_ADAPTIVE_MORPH_DEFAULT_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "servos_mix_adaptive_morph.h"
#include "conf_platform.h"


servo_mix_adaptive_morph_conf_t servo_mix_adaptive_morph_default_config =
{
	.servo_pitch = M_ADAPTIVE_MORPH_PITCH,
	.servo_roll_left = M_ADAPTIVE_MORPH_ROLL_LEFT,
	.servo_roll_right = M_ADAPTIVE_MORPH_ROLL_RIGHT,
	.servo_tail = M_ADAPTIVE_MORPH_TAIL,
	.motor = M_ADAPTIVE_MORPH_THRUST,
	
	.servo_pitch_dir = DOWN2,
	.servo_roll_left_dir = UP2,
	.servo_roll_right_dir = UP2,
	.servo_tail_dir = UP2,
	
	.min_amplitude = -1.0f,
	.max_amplitude = 1.0f,
	.min_thrust = MIN_THRUST,
	.max_thrust = MAX_THRUST,
	
	.trim_pitch = 0.0f,
	.trim_roll_left = 0.0f,
	.trim_roll_right = 0.0f,
	.trim_tail = 0.0f
};


#ifdef __cplusplus
}
#endif

#endif // SERVOS_MIX_ADAPTIVE_MORPH_DEFAULT_CONFIG_H_