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
 * \file attitude_controller_p2.h
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief A simple quaternion based proportionnal squared controller for 
 * attitude control.
 * 
 * \details It takes as input the desired attitude, the estimated attitude and 
 * the gyro rates.
 * The control scheme is the following:
 * - first the controller computes local roll pitch and yaw angular errors 
 * ( \f$ qerror \f$ ) using quaternion formulation
 * - angular errors are multiplied by a proportional gain \f$ P_{q} \f$
 * - feedback from the gyro is added to the output with gain \f$ P_{g} \f$
 * 
 * in short:
 * \f$  
 * 			output = (P_{q} . qerror) - (P_{g} . gyro)  
 * \f$
 *
 ******************************************************************************/


#ifndef ATTITUDE_CONTROLLER_P2_H_
#define ATTITUDE_CONTROLLER_P2_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "attitude_error_estimator.h"
#include "control_command.h"
#include "ahrs.h"


/**
 * \brief P^2 Attitude controller structure
 */
typedef struct 
{
	const ahrs_t* 				ahrs; 							///< Pointer to attitude estimation (input)
	const attitude_command_t* 	attitude_command;				///< Pointer to attitude command (input)
	torque_command_t*			torque_command;					///< Pointer to torque command (output)
	attitude_error_estimator_t  attitude_error_estimator;		///< Attitude error estimator
	float 						p_gain_angle[3];				///< Proportionnal gain for angular errors
	float 						p_gain_rate[3];					///< Proportionnal gain applied to gyros rates
} attitude_controller_p2_t;


/**
 * \brief P^2 Attitude controller configuration
 */
typedef struct
{
	float p_gain_angle[3];										///< Proportionnal gain for angular errors
	float p_gain_rate[3];										///< Proportionnal gain applied to gyros rates
} attitude_controller_p2_conf_t;	


/**
 * \brief               		Initialises the attitude controller structure
 * 
 * \param 	controller    		Pointer to data structure
 * \param 	config				Pointer to configuration
 * \param 	attitude_command 	Pointer to attitude command
 * \param 	ahrs		 		Pointer to the estimated attitude
 */
void attitude_controller_p2_init(attitude_controller_p2_t* controller, const attitude_controller_p2_conf_t* config, const attitude_command_t* attitude_command, torque_command_t* torque_command, const ahrs_t* ahrs);


/**
 * \brief               	Main update function
 * 
 * \param 	controller    	Pointer to data structure
 */
void attitude_controller_p2_update(attitude_controller_p2_t* controller);


#ifdef __cplusplus
}
#endif

#endif /* ATTITUDE_CONTROLLER_P2_H_ */