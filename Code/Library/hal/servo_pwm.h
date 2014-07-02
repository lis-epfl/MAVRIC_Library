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
* \file servo_pwm.h
*
* This file is the driver for the remote control
*/


#ifndef SERVO_PWM_H_
#define SERVO_PWM_H_

#include "conf_platform.h"

#define SERVO_TIMER_FREQ 1000000					///< Define the timer frequency for the servos
#define SERVO_CENTER_DUTY_MICROSEC 1500				///< Define the center of the duty cycle(?) in micro second
#define SERVO_REPEAT_FREQ 200						///< Define the period

#ifdef CS_ON_SERVO_7_8								///< to be able to use servo 7 and 8 as a normal GPIO
	#define NUMBER_OF_SERVO_OUTPUTS 6				///< free 2 last servo pin for GPIO usage
#else
	#define NUMBER_OF_SERVO_OUTPUTS 8				///< use all servo pins for PWM
#endif

/**
 * \brief Structure containing the servo output's data
 */
typedef struct {
	int value;					///< Define the PWM value of a servo
	int min;					///< Define the min value
	int max,					///< Define the max value
	int failsafe_position;		///< Define the initial(?) position of a servo
} servo_output;

/**
 * \brief Initialize the servos
 */
void init_Servos(void);

/**
 * \brief Set servos' values
 *
 * \param servo_outputs pointer to a structure containing the servos' data
 */
void set_servos(const servo_output *servo_outputs);

/**
 * \brief Set the servos' value to the failsafe value
 *
 * \param servo_outputs pointer to a structure containing the servos' data
 */
void servos_failsafe(servo_output *servo_outputs);

#endif /* SERVO_PWM_H_ */
