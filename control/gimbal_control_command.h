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
 * \file gimbal_control_command.h
 * 
 * \author MAV'RIC Team
 * \author Alexandre Cherpillod
 *   
 * \brief Data structures to hold pan and tilt command for the gimbal
 * 
 *
 ******************************************************************************/


#include "quaternions.h"

#ifndef GIMBAL_CONTROL_COMMAND_H_
#define GIMBAL_CONTROL_COMMAND_H_

#ifdef __cplusplus
	extern "C" {
#endif

/**
 * \brief 	Control command mode enum
 */
typedef enum
{
	GIMBAL_RATE_COMMAND     = 0,
	GIMBAL_ATTITUDE_COMMAND = 1
} gimbal_control_command_mode_t;

/**
 * \brief	Rate command structure
 */
typedef struct 
{	
	float xyz[2];					///<	Rate on X, Y and Z axis (in rad/s) - roll, pitch(tilt) and yaw(pan) angles
} gimbal_rate_command_t;

/**
 * \brief	Attitude command mode, either quaternion, either roll pitch and yaw angles
 */
typedef enum
{
	GIMBAL_ATTITUDE_COMMAND_MODE_QUATERNION	= 0,
	GIMBAL_ATTITUDE_COMMAND_MODE_RPY			= 1
} gimbal_attitude_command_mode_t;

/**
 * \brief	Attitude command structure
 */
typedef struct 
{	
	quat_t quat;					///<	Attitude quaternion
	float rpy[3];					///<	Roll, pitch(tilt) and yaw(pan) angles 
 	attitude_command_mode_t mode;	///< 	Command mode, defines whether the quaternion or the rpy angles should be used
} gimbal_attitude_command_t;

/**
 * \brief 	Global command structure
 */
typedef struct
{
	gimbal_control_command_mode_t  mode;		///< Control command mode
	gimbal_rate_command_t          rate;		///< Rate command
	gimbal_attitude_command_t      attitude;	///< Attitude command
} gimbal_command_t;

#ifdef __cplusplus
	}
#endif

#endif