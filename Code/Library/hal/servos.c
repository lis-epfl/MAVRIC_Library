/**
 * \page The MAV'RIC License
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */


/**
 * \file servos.c
 *
 * Abstraction layer for servomotors.
 * This module does not write to hardware, it just holds data and configuration for servos.
 */


#include "servos.h"
#include "print_util.h"
#include "time_keeper.h"

void servos_init(servos_t* servos, const servos_conf_t* config, const mavlink_stream_t* mavlink_stream)
{
	// Init dependencies
	servos->mavlink_stream = mavlink_stream;

	// Init servo array
	if ( config->servos_count <= MAX_SERVO_COUNT )
	{
		servos->servos_count = config->servos_count;
	
		for (int i = 0; i < servos->servos_count; ++i)
		{
			// Set default parameters for each type of servo
			switch ( config->types[i] )
			{
				case STANDARD_SERVO:
					servos->servo[i].trim          = 0.0f;
					servos->servo[i].min           = -1.0f;
					servos->servo[i].max           = 1.0f;
					servos->servo[i].failsafe      = 0.0f;
					servos->servo[i].repeat_freq   = 50;
					servos->servo[i].type 		   = STANDARD_SERVO;
					break;

				case MOTOR_CONTROLLER:
					servos->servo[i].trim          = 0.0f;
					servos->servo[i].min           = -1.0f;
					servos->servo[i].max           = 1.0f;
					servos->servo[i].failsafe      = -1.1f;
					servos->servo[i].repeat_freq   = 200;
					servos->servo[i].type 		   = MOTOR_CONTROLLER;
					break;

				case CUSTOM_SERVO:
					servos->servo[i].trim          = 0.0f;
					servos->servo[i].min           = -0.0f;
					servos->servo[i].max           = 0.0f;
					servos->servo[i].failsafe      = 0.0f;
					servos->servo[i].repeat_freq   = 50;
					servos->servo[i].type 		   = CUSTOM_SERVO;
					break;

				default:
					servos->servo[i].trim          = 0.0f;
					servos->servo[i].min           = -1.0f;
					servos->servo[i].max           = 1.0f;
					servos->servo[i].failsafe      = 0.0f;
					servos->servo[i].repeat_freq   = 50;
					servos->servo[i].type 		   = STANDARD_SERVO;
					break;
			}

			// Set default value to failsafe
			servos->servo[i].value = servos->servo[i].failsafe;
		}
	}
	else
	{
		servos->servos_count = 0;
		print_util_dbg_print("[SERVOS] ERROR! Too many servos\r\n");
	}
}


void servos_set_value(servos_t* servos, uint32_t servo_id, float value)
{
	if ( servo_id <= servos->servos_count )
	{
		float trimmed_value = value + servos->servo[servo_id].trim;

		if ( trimmed_value < servos->servo[servo_id].min )
		{
			servos->servo[servo_id].value = servos->servo[servo_id].min;
		}
		else if ( trimmed_value > servos->servo[servo_id].max )
		{
			servos->servo[servo_id].value = servos->servo[servo_id].max;
		}
		else
		{
			servos->servo[servo_id].value = trimmed_value;
		}
	}
}


void servos_set_value_failsafe(servos_t* servos)
{
	for (int i = 0; i < servos->servos_count; ++i)
	{
		servos->servo[i].value = servos->servo[i].failsafe;
	}
}

task_return_t servos_mavlink_send(servos_t* servos)
{
	mavlink_message_t msg;
	mavlink_msg_servo_output_raw_pack(	servos->mavlink_stream->sysid,
										servos->mavlink_stream->compid,
										&msg,
										time_keeper_get_micros(),
										0,
										(uint16_t)( 1500 + 500 * servos->servo[0].value ),
										(uint16_t)( 1500 + 500 * servos->servo[1].value ),
										(uint16_t)( 1500 + 500 * servos->servo[2].value ),
										(uint16_t)( 1500 + 500 * servos->servo[3].value ),
										(uint16_t)( 1500 + 500 * servos->servo[4].value ),
										(uint16_t)( 1500 + 500 * servos->servo[5].value ),
										(uint16_t)( 1500 + 500 * servos->servo[6].value ),
										(uint16_t)( 1500 + 500 * servos->servo[7].value )	);
	mavlink_stream_send( servos->mavlink_stream, &msg );

	return TASK_RUN_SUCCESS;
}