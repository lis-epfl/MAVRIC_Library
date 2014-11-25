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
 * \file altitude_controller_sonar.h
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief 	A simple altitude controller for copter based on sonar
 *
 ******************************************************************************/


#ifndef ALTITUDE_CONTROLLER_SONAR_H_
#define ALTITUDE_CONTROLLER_SONAR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "control_command.h"
#include "pid_controller.h"
#include "sonar.h"


/**
 * \brief Altitude controller structure
 */
typedef struct 
{
	pid_controller_t 	pid;				///< PID controller
	float				hover_point;		///< Thrust required to hover
	const sonar_t* 		sonar; 				///< Pointer to sonar sensor (input)
	thrust_command_t* 	thrust_command;		///< Pointer to thrust command (output)
} altitude_controller_sonar_t;


/**
 * \brief Altitude controller configuration
 */
typedef struct
{
	pid_controller_conf_t pid_config;		///< PID controller
	float hover_point;						///< Thrust required to hover
} altitude_controller_sonar_conf_t;	


/**
 * \brief               		Initialises the altitude controller structure
 * 
 * \param 	controller    		Pointer to data structure
 * \param 	config				Pointer to configuration
 * \param 	sonar		 		Pointer to the sonar sensor
 * \param 	thrust_command		Pointer to thrust command (output)
 */
void altitude_controller_sonar_init(altitude_controller_sonar_t* controller, const altitude_controller_sonar_conf_t* config, const sonar_t* sonar, thrust_command_t* thrust_command);

/**
 * \brief               	Main update function
 * 
 * \param 	controller    	Pointer to data structure
 */
void altitude_controller_sonar_update(altitude_controller_sonar_t* controller);


#ifdef __cplusplus
}
#endif

#endif /* ALTITUDE_CONTROLLER_SONAR_H_ */