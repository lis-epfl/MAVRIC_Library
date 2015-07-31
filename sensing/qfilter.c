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
 * \file qfilter.c
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief This file implements a complementary filter for the attitude estimation
 *
 ******************************************************************************/


#include "qfilter.h"
#include "coord_conventions.h"
#include "print_util.h" 
#include "time_keeper.h"
#include <math.h>
#include "maths.h"
#include "constants.h"

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool qfilter_init(qfilter_t* qf, const qfilter_conf_t* config, imu_t* imu, ahrs_t* ahrs)
{
	bool init_success = true;
	
	//Init dependencies
	qf->imu = imu;
	qf->ahrs = ahrs;
	
	//init qfilter gains according to provided configuration
	qf->kp = config->kp;
	qf->ki = config->ki;
	qf->kp_mag = config->kp_mag;
	qf->ki_mag = config->ki_mag;
	
	print_util_dbg_print("[QFILTER] Initialised.\r\n");
	
	return init_success;
}


void qfilter_update(qfilter_t *qf)
{
	static uint32_t convergence_update_count = 0;
	float  omc[3], omc_mag[3] , tmp[3], snorm, norm, s_acc_norm, acc_norm, s_mag_norm, mag_norm;
	quat_t qed, qtmp1, up, up_bf;
	quat_t mag_global, mag_corrected_local;
	quat_t front_vec_global = 
	{
		.s = 0.0f, 
		.v = {1.0f, 0.0f, 0.0f}
	};

	float kp 	 = qf->kp;
	float kp_mag = qf->kp_mag;
	float ki 	 = qf->ki;
	float ki_mag = qf->ki_mag;

	// Update time
	uint32_t t = time_keeper_get_time_ticks();
	float dt = time_keeper_ticks_to_seconds(t - qf->ahrs->last_update);
	qf->ahrs->dt = dt;
	qf->ahrs->last_update = t;

	// up_bf = qe^-1 *(0,0,0,-1) * qe
	up.s = 0; up.v[X] = UPVECTOR_X; up.v[Y] = UPVECTOR_Y; up.v[Z] = UPVECTOR_Z;
	up_bf = quaternions_global_to_local(qf->ahrs->qe, up);
	
	// calculate norm of acceleration vector
	s_acc_norm = qf->imu->scaled_accelero.data[X] * qf->imu->scaled_accelero.data[X] + qf->imu->scaled_accelero.data[Y] * qf->imu->scaled_accelero.data[Y] + qf->imu->scaled_accelero.data[Z] * qf->imu->scaled_accelero.data[Z];
	if ( (s_acc_norm > 0.7f * 0.7f) && (s_acc_norm < 1.3f * 1.3f) ) 
	{
		// approximate square root by running 2 iterations of newton method
		acc_norm = maths_fast_sqrt(s_acc_norm);

		tmp[X] = qf->imu->scaled_accelero.data[X] / acc_norm;
		tmp[Y] = qf->imu->scaled_accelero.data[Y] / acc_norm;
		tmp[Z] = qf->imu->scaled_accelero.data[Z] / acc_norm;

		// omc = a x up_bf.v
		CROSS(tmp, up_bf.v, omc);
	}
	else
	{
		omc[X] = 0;
		omc[Y] = 0;
		omc[Z] = 0;
	}

	// Heading computation
	// transfer 
	qtmp1 = quaternions_create_from_vector(qf->imu->scaled_compass.data); 
	mag_global = quaternions_local_to_global(qf->ahrs->qe, qtmp1);
	
	// calculate norm of compass vector
	//s_mag_norm = SQR(mag_global.v[X]) + SQR(mag_global.v[Y]) + SQR(mag_global.v[Z]);
	s_mag_norm = SQR(mag_global.v[X]) + SQR(mag_global.v[Y]);

	if ( (s_mag_norm > 0.004f * 0.004f) && (s_mag_norm < 1.8f * 1.8f) ) 
	{
		mag_norm = maths_fast_sqrt(s_mag_norm);

		mag_global.v[X] /= mag_norm;
		mag_global.v[Y] /= mag_norm;
		mag_global.v[Z] = 0.0f;   // set z component in global frame to 0

		// transfer magneto vector back to body frame 
		qf->ahrs->north_vec = quaternions_global_to_local(qf->ahrs->qe, front_vec_global);		
		
		mag_corrected_local = quaternions_global_to_local(qf->ahrs->qe, mag_global);		
		
		// omc = a x up_bf.v
		CROSS(mag_corrected_local.v, qf->ahrs->north_vec.v,  omc_mag);
		
	}
	else
	{
		omc_mag[X] = 0;
		omc_mag[Y] = 0;
		omc_mag[Z] = 0;
	}


	// get error correction gains depending on mode
	switch (qf->ahrs->internal_state)
	{
		case AHRS_UNLEVELED:
			kp = qf->kp * 10.0f;
			kp_mag = qf->kp_mag * 10.0f;
			
			ki = 0.5f * qf->ki;
			ki_mag = 0.5f * qf->ki_mag;
			
			convergence_update_count += 1;
			if( convergence_update_count > 2000 )
			{
				convergence_update_count = 0;
				qf->ahrs->internal_state = AHRS_CONVERGING;
				print_util_dbg_print("End of AHRS attitude initialization.\r\n");
			}
			break;
			
		case AHRS_CONVERGING:
			kp = qf->kp;
			kp_mag = qf->kp_mag;
			
			ki = qf->ki * 3.0f;
			ki_mag = qf->ki_mag * 3.0f;
			
			convergence_update_count += 1;
			if( convergence_update_count > 2000 )
			{
				convergence_update_count = 0;
				qf->ahrs->internal_state = AHRS_READY;
				print_util_dbg_print("End of AHRS leveling.\r\n");
			}
			break;

		case AHRS_READY:
			kp = qf->kp;
			kp_mag = qf->kp_mag;
			ki = qf->ki;
			ki_mag = qf->ki_mag;
			break;
	}

	// apply error correction with appropriate gains for accelerometer and compass

	for (uint8_t i = 0; i < 3; i++)
	{
		qtmp1.v[i] = 0.5f * (qf->imu->scaled_gyro.data[i] + kp * omc[i] + kp_mag * omc_mag[i]);
	}
	qtmp1.s = 0;

	// apply step rotation with corrections
	qed = quaternions_multiply(qf->ahrs->qe,qtmp1);

	// TODO: correct this formulas! 
	qf->ahrs->qe.s = qf->ahrs->qe.s + qed.s * dt;
	qf->ahrs->qe.v[X] += qed.v[X] * dt;
	qf->ahrs->qe.v[Y] += qed.v[Y] * dt;
	qf->ahrs->qe.v[Z] += qed.v[Z] * dt;

	snorm = qf->ahrs->qe.s * qf->ahrs->qe.s + qf->ahrs->qe.v[X] * qf->ahrs->qe.v[X] + qf->ahrs->qe.v[Y] * qf->ahrs->qe.v[Y] + qf->ahrs->qe.v[Z] * qf->ahrs->qe.v[Z];
	if (snorm < 0.0001f)
	{
		norm = 0.01f;
	}
	else
	{
		// approximate square root by running 2 iterations of newton method
		norm = maths_fast_sqrt(snorm);
	}
	qf->ahrs->qe.s /= norm;
	qf->ahrs->qe.v[X] /= norm;
	qf->ahrs->qe.v[Y] /= norm;
	qf->ahrs->qe.v[Z] /= norm;

	// bias estimate update
	qf->imu->calib_gyro.bias[X] += - dt * ki * omc[X] / qf->imu->calib_gyro.scale_factor[X];
	qf->imu->calib_gyro.bias[Y] += - dt * ki * omc[Y] / qf->imu->calib_gyro.scale_factor[Y];
	qf->imu->calib_gyro.bias[Z] += - dt * ki * omc[Z] / qf->imu->calib_gyro.scale_factor[Z];
	
	qf->imu->calib_gyro.bias[X] += - dt * ki_mag * omc_mag[X] / qf->imu->calib_compass.scale_factor[X];
	qf->imu->calib_gyro.bias[Y] += - dt * ki_mag * omc_mag[Y] / qf->imu->calib_compass.scale_factor[Y];
	qf->imu->calib_gyro.bias[Z] += - dt * ki_mag * omc_mag[Z] / qf->imu->calib_compass.scale_factor[Z];
	
	// set up-vector (bodyframe) in attitude
	qf->ahrs->up_vec.v[X] = up_bf.v[X];
	qf->ahrs->up_vec.v[Y] = up_bf.v[Y];
	qf->ahrs->up_vec.v[Z] = up_bf.v[Z];

	// Update linear acceleration
	qf->ahrs->linear_acc[X] = 9.81f * (qf->imu->scaled_accelero.data[X] - qf->ahrs->up_vec.v[X]) ;							// TODO: review this line!
	qf->ahrs->linear_acc[Y] = 9.81f * (qf->imu->scaled_accelero.data[Y] - qf->ahrs->up_vec.v[Y]) ;							// TODO: review this line!
	qf->ahrs->linear_acc[Z] = 9.81f * (qf->imu->scaled_accelero.data[Z] - qf->ahrs->up_vec.v[Z]) ;							// TODO: review this line!

	//update angular_speed.
	qf->ahrs->angular_speed[X] = qf->imu->scaled_gyro.data[X];
	qf->ahrs->angular_speed[Y] = qf->imu->scaled_gyro.data[Y];
	qf->ahrs->angular_speed[Z] = qf->imu->scaled_gyro.data[Z];
}