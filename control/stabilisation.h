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
 * \file stabilisation.h
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief Executing the PID controllers for stabilization
 *
 ******************************************************************************/


#ifndef STABILISATION_H_
#define STABILISATION_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "imu.h"
#include "pid_control.h"

/**
 * \brief	The control mode enum
 */
typedef enum 
{
	VELOCITY_COMMAND_MODE, 
	ATTITUDE_COMMAND_MODE, 
	RATE_COMMAND_MODE
} control_mode_t;

/**
 * \brief	The yaw control mode enum
 */
typedef enum     
{
	YAW_RELATIVE, 
	YAW_ABSOLUTE, 
	YAW_COORDINATED
} yaw_mode_t;

/**
 * \brief	The control command typedef
 */
typedef struct 
{
	float rpy[3];								///< roll pitch yaw rates/angles
	float thrust;								///< thrust
	float tvel[3];								///< target velocity in m/s
	float theading;								///< absolute target heading	
	control_mode_t control_mode;				///< control mode
	yaw_mode_t     yaw_mode;					///< yaw mode
} control_command_t;

/**
 * \brief	The structure used to control the vehicle with 4 PIDs
 */
typedef struct {
	pid_controller_t rpy_controller[3];			///< roll pitch yaw  controllers
	pid_controller_t thrust_controller;			///< thrust controller
	control_command_t output;					///< output
} stabiliser_t;

/**
 * \brief	Initialisation of the stabilisation module
 * \param	command				The pointer to the command structure
 */
void stabilisation_init(control_command_t *controls);

/**
 * \brief				Execute the PID controllers used for stabilization
 * 
 * \param	stabiliser	Pointer to the structure containing the PID controllers
 * \param	dt			Timestep
 * \param	errors		Array containing the errors of the controlling variables
 */
void stabilisation_run(stabiliser_t *stabiliser, float dt, float errors[]);

#ifdef __cplusplus
}
#endif

#endif /* STABILISATION_H_ */