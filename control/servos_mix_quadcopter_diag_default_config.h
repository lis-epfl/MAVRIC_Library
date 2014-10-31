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
 * \file servos_mix_quadcopter_default_config.h
 * 
 * \author MAV'RIC Team
 * \author Gregoire Heitz
 *   
 * \brief Default configuration for the servo_mix for the MAVRIC quad controlled in diag instead of cross
 *
 ******************************************************************************/


#ifndef SERVOS_MIX_QUADCOPTER_DIAG_DEFAULT_CONFIG_H_
#define SERVOS_MIX_QUADCOPTER_DIAG_DEFAULT_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "servos_mix_quadcopter_diag.h"


servo_mix_quadcopter_diag_conf_t servo_mix_quadcopter_diag_default_config =
{
	.motor_front_right		= M_FRONT_RIGHT,
	.motor_front_left		= M_FRONT_LEFT,
	.motor_rear_right		= M_REAR_RIGHT,
	.motor_rear_left		= M_REAR_LEFT,
	.motor_front_right_dir	= M_FR_DIR,
	.motor_front_left_dir	= M_FL_DIR,
	.motor_rear_right_dir	= M_RR_DIR,
	.motor_rear_left_dir	= M_RL_DIR,
	.min_thrust				= MIN_THRUST,
	.max_thrust				= MAX_THRUST
};


#ifdef __cplusplus
}
#endif

#endif // SERVOS_MIX_QUADCOPTER_DIAG_DEFAULT_CONFIG_H_