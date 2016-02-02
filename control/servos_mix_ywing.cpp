/*******************************************************************************
 * Copyright (c) 2009-2016, MAV'RIC Development Team
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
 * \file servos_mix_ywing.hpp
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief Links between torque commands and servos PWM command for Ywing
 *
 ******************************************************************************/


#include "control/servos_mix_ywing.hpp"

extern "C"
{
	#include "util/print_util.h"
}

bool servo_mix_ywing_init( servo_mix_ywing_t* mix, const servo_mix_ywing_conf_t* config, const torque_command_t* torque_command, const thrust_command_t* thrust_command, Servo* motor, Servo* flap_top, Servo* flap_right, Servo* flap_left)
{
	bool init_success = true;
	
	// Init dependencies
	mix->torque_command = torque_command;
	mix->thrust_command = thrust_command;
	mix->motor 			= motor;			
	mix->flap_top 		= flap_top;		
	mix->flap_right		= flap_right;		
	mix->flap_left 		= flap_left;		

	// Init parameters
	mix->flap_top_dir	= config->flap_top_dir;	
	mix->flap_right_dir = config->flap_right_dir;
	mix->flap_left_dir 	= config->flap_left_dir;
	mix->min_thrust		= config->min_thrust;		
	mix->max_thrust		= config->max_thrust;		
	mix->min_deflection = config->min_deflection;
	mix->max_deflection = config->max_deflection;
	
	print_util_dbg_print("[SERVOS MIX YWING] initialised \r\n");
	
	return init_success;
}


void servos_mix_ywing_update(servo_mix_ywing_t* mix)
{
	float servos[4];
	
	// Main motor
	servos[0] = mix->thrust_command->thrust;
	
	// Clip values
	if ( servos[0] < mix->min_thrust )
	{
		servos[0] = mix->min_thrust;
	}
	else if ( servos[0] > mix->max_thrust )
	{
		servos[0] = mix->max_thrust;
	}

	// Top flap
	servos[1] = mix->flap_top_dir * ( mix->torque_command->xyz[ROLL] 
									- mix->torque_command->xyz[YAW] ); 
	
	// Right flap
	servos[2]  = mix->flap_right_dir * ( mix->torque_command->xyz[ROLL] 
										+ 0.86f * mix->torque_command->xyz[PITCH] 
										+ 0.50f * mix->torque_command->xyz[YAW] );
	
	// Left flap
	servos[3]  = mix->flap_left_dir * ( mix->torque_command->xyz[ROLL] 
										- 0.86f * mix->torque_command->xyz[PITCH] 
										+ 0.50f * mix->torque_command->xyz[YAW] );
	
	// Clip values
	for (int32_t i = 1; i < 4; i++) 
	{
		if ( servos[i] < mix->min_deflection )
		{
			servos[i] = mix->min_deflection;
		}
		else if ( servos[i] > mix->max_deflection )
		{
			servos[i] = mix->max_deflection;
		}
	}

	mix->motor->write(		servos[0]);
	mix->flap_top->write(	servos[1]);
	mix->flap_right->write(	servos[2]);
	mix->flap_left->write( 	servos[3]);
}