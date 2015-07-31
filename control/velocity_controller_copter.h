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
 * \file velocity_controller_copter.h
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Nicolas Dousse
 * \author Julien Lecoeur
 *   
 * \brief A velocity controller for copters.
 * 
 * \details It takes a velocity command as input and computes an attitude 
 * command as output. It is best suited for a hovering UAV with the thrust along
 * its local -Z axis (like multicopters).
 *
 ******************************************************************************/


#ifndef VELOCITY_CONTROLLER_COPTER_H_
#define VELOCITY_CONTROLLER_COPTER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "control_command.h"
#include "ahrs.h"
#include "pid_controller.h"
#include "position_estimation.h"

/**
 * \brief Velocity controller structure
 */
typedef struct 
{
	pid_controller_t 			 pid[3];					///< PID controller for velocity along X, Y and Z in global frame
	float						 thrust_hover_point;		///< Amount of thrust required to hover (between -1 and 1)
	const ahrs_t* 				 ahrs; 						///< Pointer to attitude estimation (input)
	const position_estimation_t* pos_est;					///< Speed and position estimation (input) 
	const velocity_command_t* 	 velocity_command;			///< Pointer to velocity command (input)
	attitude_command_t* 		 attitude_command;			///< Pointer to attitude command (output)
	thrust_command_t* 		 	 thrust_command;			///< Pointer to thrust command (output)
} velocity_controller_copter_t;


/**
 * \brief Velocity controller configuration
 */
typedef struct
{
	pid_controller_conf_t 	pid_config[3];			///< Config for PID controller on velocity along X, Y and Z in global frame
	float					thrust_hover_point;		///< Amount of thrust required to hover (between -1 and 1)

} velocity_controller_copter_conf_t;	


/**
 * \brief               		Initialises the velocity controller structure
 * 
 * \param 	controller    		Pointer to data structure
 * \param 	config				Pointer to configuration
 * \param 	ahrs		 		Pointer to the estimated attitude
 * \param 	pos_est		 		Pointer to the estimated speed and position
 * \param 	velocity_command	Pointer to velocity command (input)
 * \param 	attitude_command	Pointer to attitude command (output)
 * \param 	thrust_command		Pointer to thrust command (output)
 */
void velocity_controller_copter_init(velocity_controller_copter_t* controller, const velocity_controller_copter_conf_t* config, const ahrs_t* ahrs, const position_estimation_t* pos_est, const velocity_command_t* velocity_command, attitude_command_t* attitude_command, thrust_command_t* thrust_command);


/**
 * \brief               	Main update function
 * 
 * \param 	controller    	Pointer to data structure
 */
void velocity_controller_copter_update(velocity_controller_copter_t* controller);


#ifdef __cplusplus
}
#endif

#endif /* VELOCITY_CONTROLLER_COPTER_H_ */