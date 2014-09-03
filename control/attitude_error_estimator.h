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
 * \file attitude_error_estimator.h
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief A quaternion-based attitude-error estimator, takes a reference 
 * quaternion as input and provides angular errors for roll, pitch and yaw in 
 * the local frame.
 * 
 * \details  WARNING: For optimisation purpose, the computed errors are not in 
 * radians:
 * With \f$ e_{roll} , e_{pitch}, e_{yaw} \f$ the true errors in radians around 
 * the local roll, pitch and yaw axes,
 * this estimator returns 
 * \f$ 2.sin( e_{roll} / 2 ), 2.sin( e_{pitch} / 2 ), 2.sin( e_{yaw} / 2 ) \f$
 * 
 * For small angular errors, this approximation is sensible.
 *
 ******************************************************************************/


#ifndef ATTITUDE_ERROR_ESTIMATOR_H_
#define ATTITUDE_ERROR_ESTIMATOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "quaternions.h"
#include "coord_conventions.h"
#include "ahrs.h"

/**
 * \brief Quaternion attitude error estimator data structure
 */
typedef struct 
{
	quat_t quat_ref;			///<	Reference attitude, the errors in roll pitch and yaw will be computed relative to this reference
	float rpy_errors[3];		///<	Local errors: roll, pitch and yaw
	const ahrs_t* ahrs;			///<	Pointer to AHRS (current attitude), must be updated externally
} attitude_error_estimator_t;


/**
 * \brief               	Initialises the attitude error estimator structure
 * 
 * \param 	estimator    	Pointer to data structure
 * \param 	ahrs		 	Pointer to the estimated attitude
 */
void attitude_error_estimator_init(attitude_error_estimator_t* estimator, const ahrs_t* ahrs);


/**
 * \brief               	Updates the reference attitude
 * 
 * \param 	estimator    	Pointer to data structure
 * \param 	quat_ref      	New attitude quaternion to be used as reference
 */
void attitude_error_estimator_set_quat_ref(attitude_error_estimator_t* estimator, const quat_t quat_ref);


/**
 * \brief               	Function to update the reference attitude from Euler angles
 * 
 * \param 	estimator    	Pointer to data structure
 * \param 	aero          	Roll, pitch and yaw angle given in aero_attitude_t structure (radians)
 */
void attitude_error_estimator_set_quat_ref_from_aero(attitude_error_estimator_t* estimator, const aero_attitude_t aero);


/**
 * \brief               	Function to update the reference attitude from Euler angles
 * 
 * \param 	estimator    	Pointer to data structure
 * \param 	rpy           	Roll, pitch and yaw angle (radians)
 */
void attitude_error_estimator_set_quat_ref_from_rpy(attitude_error_estimator_t* estimator, const float rpy[3]);


/**
 * \brief               	Main update function, computes the local angular errors is roll, pitch and yaw
 * 
 * \param 	estimator    	Pointer to data structure
 */
void attitude_error_estimator_update(attitude_error_estimator_t* estimator);


#ifdef __cplusplus
}
#endif

#endif /* ATTITUDE_ERROR_ESTIMATOR_H_ */