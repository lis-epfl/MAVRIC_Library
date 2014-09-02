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
 * \file servos_mix_quadcopter_cross.h
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief Links between torque commands and servos PWM command for quadcopters 
 * in cross configuration
 *
 ******************************************************************************/


#ifndef SERVOS_MIX_QUADCOPTER_CROSS_H_
#define SERVOS_MIX_QUADCOPTER_CROSS_H_

#ifdef __cplusplus
	extern "C" {
#endif


#include "control_command.h"
#include "servos.h"


typedef enum
{
	CW 	= 1,
	CCW	= -1
} rot_dir_t;


typedef struct
{
	uint8_t 	motor_front;
	uint8_t 	motor_left;
	uint8_t 	motor_right;
	uint8_t		motor_rear;
	rot_dir_t 	motor_front_dir;
	rot_dir_t 	motor_left_dir;
	rot_dir_t 	motor_right_dir;
	rot_dir_t 	motor_rear_dir;
	float 		min_thrust;
	float		max_thrust;
} servo_mix_quadcopter_cross_conf_t;


/**
 * \brief	servos mix structure
 */
typedef struct 
{	
	uint8_t   	motor_front;
	uint8_t   	motor_left;
	uint8_t   	motor_right;
	uint8_t   	motor_rear;
	rot_dir_t 	motor_front_dir;
	rot_dir_t 	motor_left_dir;
	rot_dir_t 	motor_right_dir;
	rot_dir_t 	motor_rear_dir;
	float 		min_thrust;
	float		max_thrust;
	const torque_command_t* torque_command;
	const thrust_command_t* thrust_command;
	servos_t*          		servos;
} servo_mix_quadcotper_cross_t;


/**
 * @brief [brief description]
 * @details [long description]
 * 
 * @param servo_mix [description]
 * @param config [description]
 * @param torque_command [description]
 * @param servo_pwm [description]
 */
void servo_mix_quadcotper_cross_init(servo_mix_quadcotper_cross_t* mix, const servo_mix_quadcopter_cross_conf_t* config, const torque_command_t* torque_command, const thrust_command_t* thrust_command, servos_t* servos);


/**
 * @brief [brief description]
 * @details [long description]
 * 
 * @param servo_mix [description]
 */
void servos_mix_quadcopter_cross_update(servo_mix_quadcotper_cross_t* mix);


#ifdef __cplusplus
	}
#endif

#endif