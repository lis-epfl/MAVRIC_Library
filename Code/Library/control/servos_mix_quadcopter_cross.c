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
 * \file servos_mix_quadcopter_cross.h
 *
 * Links between torque commands and servos PWM command for quadcopters in cross configuration
 */


// TODO: update documentation


#include "servos_mix_quadcopter_cross.h"


void servo_mix_quadcotper_cross_init(servo_mix_quadcotper_cross_t* mix, const servo_mix_quadcopter_cross_conf_t* config, const torque_command_t* torque_command, const thrust_command_t* thrust_command, servos_t* servos)
{
	// Init dependencies
	mix->torque_command = torque_command;
	mix->thrust_command = thrust_command;
	mix->servos      	= servos;

	// Init parameters
	mix->motor_front     = config->motor_front;
	mix->motor_left      = config->motor_left;
	mix->motor_right     = config->motor_right;
	mix->motor_rear      = config->motor_rear;

	mix->motor_front_dir = config->motor_front_dir;
	mix->motor_left_dir  = config->motor_left_dir;
	mix->motor_right_dir = config->motor_right_dir;	
	mix->motor_rear_dir  = config->motor_rear_dir;

	mix->min_thrust 	   = config->min_thrust;
	mix->max_thrust 	   = config->max_thrust;
}


void servos_mix_quadcopter_cross_update(servo_mix_quadcotper_cross_t* mix)
{
	int32_t i;
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

	servos_set_value(mix->servos, mix->motor_front, motor[0]);
	servos_set_value(mix->servos, mix->motor_right, motor[1]);
	servos_set_value(mix->servos, mix->motor_rear,  motor[2]);
	servos_set_value(mix->servos, mix->motor_left,  motor[3]);
}