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
 * \file servos.h
 *
 * Abstraction layer for servomotors
 * This module does not write to hardware, it just holds data and configuration for servos.
 */


#ifndef SERVOS_H_
#define SERVOS_H_

#ifdef __cplusplus
	extern "C" {
#endif

#include <stdint.h>
#include "mavlink_stream.h"
#include "scheduler.h"

#define MAX_SERVO_COUNT 8

typedef enum
{
	STANDARD_SERVO 	 = 0,
	MOTOR_CONTROLLER = 1,
	CUSTOM_SERVO     = 2
} servo_type_t;


typedef struct
{
	uint32_t servos_count;
	servo_type_t types[MAX_SERVO_COUNT];
} servos_conf_t;


typedef struct
{
	float value;			///< Normalized value of the servo (between -1 and 1)
	float trim;				///< Trim value (between -1 and 1)
	float min;				///< Minimum value (between -1 and 1)
	float max;				///< Max value (between -1 and 1)
	float failsafe;			///< Failsafe position of the servo (between -1 and 1)
	uint32_t repeat_freq;	///< Update frequency of the servo (in Hz)
	servo_type_t type;		///< Type of servo
} servo_entry_t;


typedef struct 
{
	uint32_t servos_count;
	servo_entry_t servo[MAX_SERVO_COUNT];
	const mavlink_stream_t* mavlink_stream;
} servos_t;


void servos_init(servos_t* servos, const servos_conf_t* config, const mavlink_stream_t* mavlink_stream);


void servos_set_value(servos_t* servos, uint32_t servo_id, float value);


void servos_set_value_failsafe(servos_t* servos);


task_return_t servos_mavlink_send(servos_t* servos);


#ifdef __cplusplus
	}
#endif

#endif