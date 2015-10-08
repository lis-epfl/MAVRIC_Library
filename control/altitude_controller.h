/*******************************************************************************
 * Copyright (c) 2009-2015, MAV'RIC Development Team
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
 * \file altitude_controller.h
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief 	A simple altitude controller for copter
 *
 ******************************************************************************/


#ifndef ALTITUDE_CONTROLLER_H_
#define ALTITUDE_CONTROLLER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "control_command.h"
#include "pid_controller.h"
#include "altitude.h"


/**
 * \brief Altitude controller structure
 */
typedef struct 
{
	pid_controller_t			pid;				///< Controller
	float						hover_point;		///< Thrust required to hover
	const position_command_t*	position_command;	///< Pointer to altitude command (input)
	const altitude_t* 			altitude_estimated; ///< Pointer to estimated altitude (input)
	thrust_command_t* 			thrust_command;		///< Pointer to thrust command (output)
} altitude_controller_t;


/**
 * \brief Altitude controller configuration
 */
typedef struct
{
	float 					hover_point;		///< Thrust required to hover
	pid_controller_conf_t 	pid_config;			///< Proportionnal gain
} altitude_controller_conf_t;	


/**
 * \brief               		Initializes the altitude controller structure
 * 
 * \param 	controller    		Pointer to data structure
 * \param 	config				Pointer to configuration
 * \param 	position_command	Pointer to the position command
 * \param 	altitude_estimated	Pointer to the estimated altitude
 * \param 	thrust_command		Pointer to thrust command (output)
 */
void altitude_controller_init(	altitude_controller_t* controller, 
								const altitude_controller_conf_t* config, 
								const position_command_t* position_command, 
								const altitude_t* altitude_estimated, 
								thrust_command_t* thrust_command);


/**
 * \brief               	Main update function
 * 
 * \param 	controller    	Pointer to data structure
 */
void altitude_controller_update(altitude_controller_t* controller);


#ifdef __cplusplus
}
#endif

#endif /* ALTITUDE_CONTROLLER_H_ */