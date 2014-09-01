/** 
 * \page The MAV'RIC license
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */
 
 
/**
 * \file servos_mix_quadcopter_diag.h
 *
 * Links between torque commands and servos PWM command for quadcopters in diagonal configuration
 */


// TODO: update documentation


#include "servos_mix_quadcopter_diag.h"


void servo_mix_quadcotper_diag_init(servo_mix_quadcotper_diag_t* mix, const servo_mix_quadcopter_diag_conf_t* config, const torque_command_t* torque_command, const thrust_command_t* thrust_command, servos_t* servos)
{
	// Init dependencies
	mix->torque_command = torque_command;
	mix->thrust_command = thrust_command;
	mix->servos      	= servos;

	// Init parameters
	mix->motor_front_right     = config->motor_front_right;
	mix->motor_front_left      = config->motor_front_left;
	mix->motor_rear_right      = config->motor_rear_right;
	mix->motor_rear_left       = config->motor_rear_left;

	mix->motor_front_right_dir = config->motor_front_right_dir;
	mix->motor_front_left_dir  = config->motor_front_left_dir;
	mix->motor_rear_right_dir  = config->motor_rear_right_dir;	
	mix->motor_rear_left_dir   = config->motor_rear_left_dir;

	mix->min_thrust 	   = config->min_thrust;
	mix->max_thrust 	   = config->max_thrust;
}


void servos_mix_quadcopter_diag_update(servo_mix_quadcotper_diag_t* mix)
{
	int32_t i;
	float motor[4];
	
	// Front Right motor
	motor[0] = 	mix->thrust_command->thrust + 
				( - mix->torque_command->xyz[0] ) +
				( + mix->torque_command->xyz[1] ) + 
				mix->motor_front_right_dir * mix->torque_command->xyz[2];

	// Front Left motor
	motor[1] = 	mix->thrust_command->thrust +
				( + mix->torque_command->xyz[0] ) +
				( + mix->torque_command->xyz[1] ) + 
				mix->motor_front_left_dir * mix->torque_command->xyz[2];
	
	// Rear Right motor
	motor[2]  = mix->thrust_command->thrust +
				(- mix->torque_command->xyz[0]) +
				(- mix->torque_command->xyz[1]) +
				mix->motor_rear_right_dir * mix->torque_command->xyz[2];
	
	// Rear Left motor
	motor[3]  = mix->thrust_command->thrust + 
				( + mix->torque_command->xyz[0] ) + 
				( - mix->torque_command->xyz[1] ) + 
				mix->motor_rear_left_dir * mix->torque_command->xyz[2];
	
	// Clip values
	for (i=0; i<4; i++) 
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

	servos_set_value(mix->servos, mix->motor_front_right, motor[0]);
	servos_set_value(mix->servos, mix->motor_front_left,  motor[1]);
	servos_set_value(mix->servos, mix->motor_rear_right,  motor[2]);
	servos_set_value(mix->servos, mix->motor_rear_left,   motor[3]);
}