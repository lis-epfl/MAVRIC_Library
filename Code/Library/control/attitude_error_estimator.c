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
 * \file attitude_error_estimator.c
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


#include "attitude_error_estimator.h"
#include "coord_conventions.h"

void attitude_error_estimator_init(attitude_error_estimator_t* estimator, const ahrs_t* ahrs)
{
	// Init ref to quat_attitude
	estimator->ahrs = ahrs;

	// Init quat_ref
	estimator->quat_ref.s = 1;
	estimator->quat_ref.v[0] = 0;
	estimator->quat_ref.v[1] = 0;
	estimator->quat_ref.v[2] = 0;

	// Init output
	estimator->rpy_errors[0] = 0;
	estimator->rpy_errors[1] = 0;
	estimator->rpy_errors[2] = 0;
}


void attitude_error_estimator_set_quat_ref(attitude_error_estimator_t* estimator, const quat_t quat_ref)
{
	estimator->quat_ref = quat_ref;
}


void attitude_error_estimator_set_quat_ref_from_aero(attitude_error_estimator_t* estimator, const aero_attitude_t aero)
{
	estimator->quat_ref = coord_conventions_quaternion_from_aero(aero);
}


void attitude_error_estimator_set_quat_ref_from_rpy(attitude_error_estimator_t* estimator, const float rpy[3])
{
	aero_attitude_t aero;
	aero.rpy[0] = rpy[0];
	aero.rpy[1] = rpy[1];
	aero.rpy[2] = rpy[2];

	estimator->quat_ref = coord_conventions_quaternion_from_aero(aero);
}


void attitude_error_estimator_update(attitude_error_estimator_t* estimator)
{
	quat_t quat_attitude = estimator->ahrs->qe;
	quat_t quat_ref = estimator->quat_ref;
	quat_t quat_error;

	// Compute quaternioin error in global frame
	quat_error = quaternions_multiply(quat_ref, quaternions_inverse(quat_attitude));
	
	// Express error in local coordinates
	quat_error = quaternions_multiply(	quaternions_inverse(quat_attitude),
										quaternions_multiply(quat_error, 
															 quat_attitude)  );

	// Find shortest rotation
	if (quat_error.s < 0)
	{
		quat_error = quaternions_inverse(quat_error);
	}

	// Approximate roll pitch and yaw errors with quat_error vector part
	estimator->rpy_errors[0] = 2 * quat_error.v[0];
	estimator->rpy_errors[1] = 2 * quat_error.v[1];
	estimator->rpy_errors[2] = 2 * quat_error.v[2];
}
