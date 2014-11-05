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
 * \file attitude_controller.h
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief A cascaded controller for attitude & rate control.
 * 
 * \details It takes a command in attitude (roll/pitch/yaw or quaternion) as
 * input, and computes a torque command on roll pitch and yaw.
 * The inner PID loop controls the angular speed around the 3 axis. This inner 
 * loop is fed by the outer PID loop which controls the attitude. 
 * The error of the outer loop is computed using quaternion arithmetic, and thus
 * avoids gimbal locks as long as the attitude error is smaller than 90 degrees 
 * on the pitch axis.  
 *
 ******************************************************************************/


#ifndef ATTITUDE_CONTROLLER_H_
#define ATTITUDE_CONTROLLER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "attitude_error_estimator.h"
#include "control_command.h"
#include "ahrs.h"
#include "pid_controller.h"

/**
 * \brief Control mode (attitude or rate)
 */
typedef enum
{
	ATTITUDE_CONTROLLER_MODE_DEFAULT 	= 0,	///< Angles + rate control
	ATTITUDE_CONTROLLER_MODE_RATE_ONLY	= 1,	///< Rate control only
} attitude_controller_mode_t;


/**
 * \brief Attitude controller structure
 */
typedef struct 
{
	attitude_error_estimator_t  attitude_error_estimator;	///< Attitude error estimator
	pid_controller_t 			rate_pid[3];				///< Angular rate PID controller for roll, pitch and yaw
	pid_controller_t 			angle_pid[3];				///< Attitude PID controller for roll, pitch and yaw
	attitude_controller_mode_t 	mode;						///< Control mode: Angle (default), or rate
	const ahrs_t* 				ahrs; 						///< Pointer to attitude estimation (input)
	const attitude_command_t* 	attitude_command;			///< Pointer to attitude command (input)
	rate_command_t* 			rate_command;				///< Pointer to rate command (input/output)
	torque_command_t* 			torque_command;				///< Pointer to torque command (output)
} attitude_controller_t;


/**
 * \brief Attitude controller configuration
 */
typedef struct
{
	pid_controller_conf_t rate_pid_config[3];	///< Angular rate PID controller for roll, pitch and yaw
	pid_controller_conf_t angle_pid_config[3];	///< Attitude PID controller for roll, pitch and yaw
} attitude_controller_conf_t;	


/**
 * \brief               		Initialises the attitude controller structure
 * 
 * \param 	controller    		Pointer to data structure
 * \param 	config				Pointer to configuration
 * \param 	ahrs		 		Pointer to the estimated attitude
 * \param 	attitude_command	Pointer to attitude command (input)
 * \param 	rate_command		Pointer to rate command (input/output)
 * \param 	torque_command		Pointer to torque command (output)
 */
void attitude_controller_init(attitude_controller_t* controller, const attitude_controller_conf_t* config, const ahrs_t* ahrs, attitude_command_t* attitude_command, rate_command_t* rate_command, torque_command_t* torque_command);


/**
 * \brief               	Main update function
 * 
 * \param 	controller    	Pointer to data structure
 */
void attitude_controller_update(attitude_controller_t* controller);


#ifdef __cplusplus
}
#endif

#endif /* ATTITUDE_CONTROLLER_H_ */