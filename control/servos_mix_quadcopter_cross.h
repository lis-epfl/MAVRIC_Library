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


/**
 * \brief Enumerate the turn direction of a motor
 */
typedef enum
{
	CW 	= 1,					///< Clock wise
	CCW	= -1					///< Counter Cleck wise
} rot_dir_t;


/**
 * \brief The servo mix structure for a quad in cross shape
 */
typedef struct
{
	uint8_t 	motor_front;		///< Front motor
	uint8_t 	motor_left;			///< Left  motor
	uint8_t 	motor_right;		///< Right motor
	uint8_t		motor_rear;			///< Rear  motor
	rot_dir_t 	motor_front_dir;	///< Front motor turning direction
	rot_dir_t 	motor_left_dir;		///< Left  motor turning direction
	rot_dir_t 	motor_right_dir;	///< Right motor turning direction
	rot_dir_t 	motor_rear_dir;		///< Rear  motor turning direction
	float 		min_thrust;			///< Minimum thrust
	float		max_thrust;			///< Maximum thrust
} servo_mix_quadcopter_cross_conf_t;


/**
 * \brief	servos mix structure
 */
typedef struct 
{	
	uint8_t   	motor_front;					///< Front motor
	uint8_t   	motor_left;						///< Left  motor
	uint8_t   	motor_right;					///< Right motor
	uint8_t   	motor_rear;						///< Rear  motor
	rot_dir_t 	motor_front_dir;				///< Front motor turning direction
	rot_dir_t 	motor_left_dir;					///< Left  motor turning direction
	rot_dir_t 	motor_right_dir;				///< Right motor turning direction
	rot_dir_t 	motor_rear_dir;					///< Rear  motor turning direction
	float 		min_thrust;						///< Minimum thrust
	float		max_thrust;						///< Maximum thrust
	const torque_command_t* torque_command;		///< Pointer to the torque command structure
	const thrust_command_t* thrust_command;		///< Pointer to the thrust command structure
	servos_t*          		servos;				///< Pointer to the servos structure
} servo_mix_quadcotper_cross_t;


/**
 * @brief		Initialize the servo mix
 * 
 * @param mix				Pointer to the servo mix structure of the quad in cross shape
 * @param config			Pointer to the configuration of servo mix structure
 * @param torque_command	Pointer to the torque command structure
 * @param thrust_command	Pointer to the thrust command structure
 * @param servos			Pointer to the servos structure
 */
void servo_mix_quadcotper_cross_init(	servo_mix_quadcotper_cross_t* mix, 
										const servo_mix_quadcopter_cross_conf_t* config, 
										const torque_command_t* torque_command, 
										const thrust_command_t* thrust_command, 
										servos_t* servos);


/**
 * @brief			Update des servos mix
 * 
 * @param mix		Pointer to the servos mix structure
 */
void servos_mix_quadcopter_cross_update(servo_mix_quadcotper_cross_t* mix);


#ifdef __cplusplus
	}
#endif

#endif