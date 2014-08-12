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
#include <stdbool.h>
#include "servos.h"


/**
 * \brief Initialize the hardware line for servos
 */
void pwm_servos_init(bool use_servos_7_8);


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
