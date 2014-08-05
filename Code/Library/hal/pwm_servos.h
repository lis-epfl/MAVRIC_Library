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
* \file pwm_servos.h
*
* This file is the driver for pwm servos
*/


#ifndef PWM_SERVOS_H_
#define PWM_SERVOS_H_

#ifdef __cplusplus
	extern "C" {
#endif

#include <stdint.h>
#include "servos.h"

#define SERVO_TIMER_FREQ 1000000					///< Define the timer frequency for the servos
#define SERVO_CENTER_DUTY_MICROSEC 1500				///< Define the center of the duty cycle(?) in micro second
#define SERVO_REPEAT_FREQ 200						///< Define the period

#ifndef CS_ON_SERVO_7_8								///< to be able to use servo 7 and 8 as a normal GPIO
	#define NUMBER_OF_SERVO_OUTPUTS 8				///< use all servo pins for PWM
#else
	#define NUMBER_OF_SERVO_OUTPUTS 6				///< free 2 last servo pin for GPIO usage
#endif





/**
 * \brief Initialize the hardware line for servos
 */
void pwm_servos_init(void);


/**
 * \brief Set servos' values
 *
 * \param servo_outputs pointer to a structure containing the servos' data
 */
void pwm_servos_write_to_hardware(const servos_t* servos);


#ifdef __cplusplus
	}
#endif

#endif /* PWM_SERVOS_H_ */
