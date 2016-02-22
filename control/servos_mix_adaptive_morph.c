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
 * \file servos_mix_adaptive_morph.c
 * 
 * \author MAV'RIC Team
 * \author Simon Pyroth
 *   
 * \brief Links between regulation output and PWM commands for a adaptive_morph aircraft
 *
 ******************************************************************************/


#include "servos_mix_adaptive_morph.h"
#include "print_util.h"
#include "constants.h"

bool servo_mix_adaptive_morph_init(servo_mix_adaptive_morph_t* mix, const servo_mix_adaptive_morph_conf_t* config, control_command_t* command, servos_t* servos, remote_t* remote)
{
	bool init_success = true;
	
	// Init dependencies
	mix->command		= command;
	mix->servos      	= servos;
	mix->remote			= remote;

	// Init parameters
	mix->config.servo_elevator			= config->servo_elevator;
	mix->config.servo_wing_left		= config->servo_wing_left;
	mix->config.servo_wing_right	= config->servo_wing_right;
	mix->config.servo_tail			= config->servo_tail;
	mix->config.motor				= config->motor;
	
	mix->config.servo_elevator_dir = config->servo_elevator_dir;
	mix->config.servo_wing_left_dir	= config->servo_wing_left_dir;
	mix->config.servo_wing_right_dir	= config->servo_wing_right_dir;
	mix->config.servo_tail_dir = config->servo_tail_dir;
	
	mix->config.min_amplitude	= config->min_amplitude;
	mix->config.max_amplitude	= config->max_amplitude;
	mix->config.min_thrust		= config->min_thrust;
	mix->config.max_thrust		= config->max_thrust;
	
	mix->config.trim_pitch		= config->trim_pitch;
	mix->config.trim_roll_left		= config->trim_roll_left;
	mix->config.trim_roll_right		= config->trim_roll_right;
	mix->config.trim_tail		= config->trim_tail;
	
	mix->config.debug.is_pitch_control = config->debug.is_pitch_control;
	mix->config.debug.is_single_folding = config->debug.is_single_folding;
	
	// Debug and return
	print_util_dbg_print("[SERVOS MIX WING] initialised \r\n");
	return init_success;
}


void servos_mix_adaptive_morph_update(servo_mix_adaptive_morph_t* mix)
{
	servos_mix_adaptive_morph_update_command(mix, mix->command);
}

void servos_mix_adaptive_morph_update_command(servo_mix_adaptive_morph_t* mix, control_command_t* command)
{
	// Calculate value to be sent to the motors
	float tmp_pitch				= mix->config.servo_elevator_dir * ( (command->rpy[PITCH] + mix->config.trim_pitch) );
	float tmp_wing_left_servo	= mix->config.servo_wing_left_dir  * ( (command->rpy[ROLL] + mix->config.trim_roll_left) );
	float tmp_wing_right_servo	= mix->config.servo_wing_right_dir  * ( (command->rpy[ROLL] + mix->config.trim_roll_right) );
	float tmp_tail_servo		= mix->config.servo_tail_dir  * ( (command->rpy[YAW] + mix->config.trim_tail) );
	float tmp_motor				= command->thrust;
	
	// Clip values
	if (tmp_pitch < mix->config.min_amplitude)
	{
		tmp_pitch = mix->config.min_amplitude;
	}
	else if (tmp_pitch > mix->config.max_amplitude)
	{
		tmp_pitch = mix->config.max_amplitude;
	}
	
	if (tmp_wing_left_servo < mix->config.min_amplitude)
	{
		tmp_wing_left_servo = mix->config.min_amplitude;
	}
	else if (tmp_wing_left_servo > mix->config.max_amplitude)
	{
		tmp_wing_left_servo = mix->config.max_amplitude;
	}
	
	if (tmp_wing_right_servo < mix->config.min_amplitude)
	{
		tmp_wing_right_servo = mix->config.min_amplitude;
	}
	else if (tmp_wing_right_servo > mix->config.max_amplitude)
	{
		tmp_wing_right_servo = mix->config.max_amplitude;
	}
	
	if (tmp_tail_servo < mix->config.min_amplitude)
	{
		tmp_tail_servo = mix->config.min_amplitude;
	}
	else if (tmp_tail_servo > mix->config.max_amplitude)
	{
		tmp_tail_servo = mix->config.max_amplitude;
	}
	
	if (tmp_motor < mix->config.min_thrust)
	{
		tmp_motor = mix->config.min_thrust;
	}
	else if (tmp_motor > mix->config.max_thrust)
	{
		tmp_motor = mix->config.max_thrust;
	}

	// Set the calculated values to each motors
	servos_set_value(mix->servos, mix->config.servo_wing_right, tmp_wing_right_servo);
	servos_set_value(mix->servos, mix->config.servo_tail, tmp_tail_servo);
	servos_set_value(mix->servos, mix->config.motor, tmp_motor);
	servos_set_value(mix->servos, mix->config.servo_elevator, tmp_pitch);
	servos_set_value(mix->servos, mix->config.servo_wing_left, tmp_wing_left_servo);
	servos_set_value(mix->servos, 6, tmp_tail_servo);	// This one is used only for the third one to work properly...
}