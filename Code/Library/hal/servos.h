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
	uint32_t pwm_neutral;	///< Neutral PWM signal (ex: PWM between 550 and 1450 => pwm_neutral = 1000)
	uint32_t pwm_amplitude;	///< Amplitude of PWM signal (ex: between 550 and 1450 => pwm_magnitude = 450)
	servo_type_t type;		///< Type of servo
} servo_entry_t;


typedef struct 
{
	uint32_t servos_count;
	servo_entry_t servo[MAX_SERVO_COUNT];
} servos_t;


void servos_init(servos_t* servos, const servos_conf_t* config);


void servos_set_value(servos_t* servos, uint32_t servo_id, float value);


void servos_failsafe(servos_t* servos);


#ifdef __cplusplus
	}
#endif

#endif