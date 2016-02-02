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
 * \file servos_mix_quadcopter_cross.cpp
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief Links between torque commands and servos PWM command for quadcopters 
 * in cross configuration
 *
 ******************************************************************************/


#include "servos_mix_quadcopter_cross.hpp"


bool servos_mix_quadcotper_cross_init(	servos_mix_quadcotper_cross_t* mix, 
										const servos_mix_quadcopter_cross_conf_t* config, 
										const torque_command_t* torque_command, 
										const thrust_command_t* thrust_command, 
										Servo* motor_front,		
										Servo* motor_left,			
										Servo* motor_right,		
										Servo* motor_rear)
{
	// Init dependencies
	mix->torque_command = torque_command;
	mix->thrust_command = thrust_command;
	mix->motor_front    = motor_front;
	mix->motor_left     = motor_left;
	mix->motor_right    = motor_right;
	mix->motor_rear     = motor_rear;

	// Init parameters
	mix->motor_front_dir = config->motor_front_dir;
	mix->motor_left_dir  = config->motor_left_dir;
	mix->motor_right_dir = config->motor_right_dir;	
	mix->motor_rear_dir  = config->motor_rear_dir;

	mix->min_thrust 	   = config->min_thrust;
	mix->max_thrust 	   = config->max_thrust;

	return true;
}


bool servos_mix_quadcopter_cross_update(servos_mix_quadcotper_cross_t* mix)
{
	float motor[4];
	
	// Front motor
	motor[0] = 	mix->thrust_command->thrust + 
				mix->torque_command->xyz[1]  + 
				mix->motor_front_dir * mix->torque_command->xyz[2];

	// Right motor
	motor[1] = 	mix->thrust_command->thrust +
				(- mix->torque_command->xyz[0]) + 
				mix->motor_right_dir * mix->torque_command->xyz[2];
	
	// Rear motor
	motor[2]  = mix->thrust_command->thrust +
				(- mix->torque_command->xyz[1]) +
				mix->motor_rear_dir * mix->torque_command->xyz[2];
	
	// Left motor
	motor[3]  = mix->thrust_command->thrust + 
				mix->torque_command->xyz[0]  + 
				mix->motor_left_dir * mix->torque_command->xyz[2];
	
	// Clip values
	for (int32_t i = 0; i < 4; i++) 
	{
		if ( motor[i] < mix->min_thrust )
		{
			motor[i] = mix->min_thrust;
		}
		else if ( motor[i] > mix->max_thrust )
		{
			motor[i] = mix->max_thrust;
		}
	}

	mix->motor_front->write(motor[0]);
	mix->motor_right->write(motor[1]);
	mix->motor_rear->write( motor[2]);
	mix->motor_left->write( motor[3]);

	return true;
}