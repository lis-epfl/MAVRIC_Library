/*******************************************************************************
 * Copyright (c) 2009-2016, MAV'RIC Development Team
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
 * \file stabilisation_ywing.h
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief Ywing stabilisation
 *
 ******************************************************************************/


/**
 *   Disclaimer: this WIP
 */


#ifndef STABILISATION_YWING_H_
#define STABILISATION_YWING_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "quaternions.h"

#include "control_command.h"
#include "attitude_controller.h"
#include "servos_mix_ywing.h"
#include "remote.h"


/**
 * \brief Flight modes for ywing
 */
 typedef enum
 {
 	YWING_DISARMED 		= 0,		///< Disarmed
 	YWING_MANUAL		= 1,		///< Armed
 	YWING_RATE			= 2, 		///< Rate control only
 	YWING_HOVER			= 3,		///< Attitude + rate control in hover
 	YWING_TRANSITION 	= 4,		///< Attitude + rate control in transition
 	YWING_45			= 5, 		///< Attitude + rate control with base attitude at 45 degrees
 } ywing_mode_t;


/**
 * \brief Ywing
 */
typedef struct
{
	ywing_mode_t 			mode;					///< Current flight mode
	command_t 				command;				///< Input command
	attitude_controller_t 	attitude_controller;	///< Attitude controler
	float 					reference_pitch;		///< Current pitch reference
	float 					reference_heading;		///< Current heading reference
	float 					reference_roll;			///< Current roll reference
	const ahrs_t*			ahrs;					///< Pointer to attitude estimation (input)
	const remote_t*			remote;					///< Pointer to remote (input)
} stabilisation_ywing_t;


/**
 * \brief Configuration for the ywing structure
 */
typedef struct  
{
} stabilisation_ywing_conf_t;




/**
 * \brief  	Initialisation Ywing 
 * 
 * \param 	stabilisation_ywing 	Pointer to data struct
 * \param 	config 					Configuration
 * 
 * \return 	True if successful, false if not
 */
bool stabilisation_ywing_init(stabilisation_ywing_t* stabilisation_ywing, stabilisation_ywing_conf_t* config);


/**
 * \brief 	Main update function
 * 
 * \param 	stabilisation_ywing 	Pointer to data struct
 */
void stabilisation_ywing_update(stabilisation_ywing_t* stabilisation_ywing);


#ifdef __cplusplus
}
#endif

#endif /* STABILISATION_YWING_H_ */
