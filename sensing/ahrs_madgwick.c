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
 * \file ahrs_madgwick.c
 * 
 * \author MAV'RIC Team
 * \author SOH Madgwick
 * \author Julien Lecoeur
 *   
 * \brief Implementation of Madgwick's AHRS algorithms.
 *
 * See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
 *
 * Date			Author          Notes
 * 29/09/2011	SOH Madgwick    Initial release
 * 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
 * 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
 * 04/02/2014	Julien Lecoeur	Adapt to MAVRIC
 *
 ******************************************************************************/


/**
 *   Disclaimer: this WIP
 */



#include "ahrs_madgwick.h"
#include "maths.h"
#include "print_util.h"
#include "constants.h"
#include "time_keeper.hpp"
#include "quaternions.h"


bool ahrs_madgwick_init(ahrs_madgwick_t* ahrs_madgwick, const ahrs_madgwick_conf_t* config, imu_t* imu, ahrs_t* ahrs)
{
	// Init dependencies
	ahrs_madgwick->imu 	= imu;
	ahrs_madgwick->ahrs = ahrs;

	// Init variables
	ahrs_madgwick->ref_b[X] 	= 1.0f;
	ahrs_madgwick->ref_b[Y] 	= 0.0f;
	ahrs_madgwick->ref_b[Z] 	= 0.0f;

	// Init ahrs structure
	ahrs_madgwick->ahrs->qe.s = 1.0f;
	ahrs_madgwick->ahrs->qe.v[X] = 0.0f;
	ahrs_madgwick->ahrs->qe.v[Y] = 0.0f;
	ahrs_madgwick->ahrs->qe.v[Z] = 0.0f;
	
	ahrs_madgwick->ahrs->angular_speed[X] = 0.0f;
	ahrs_madgwick->ahrs->angular_speed[Y] = 0.0f;
	ahrs_madgwick->ahrs->angular_speed[Z] = 0.0f;
	
	ahrs_madgwick->ahrs->linear_acc[X] = 0.0f;
	ahrs_madgwick->ahrs->linear_acc[Y] = 0.0f;
	ahrs_madgwick->ahrs->linear_acc[Z] = 0.0f;
	
	ahrs_madgwick->ahrs->north_vec.s    = 0.0f;
	ahrs_madgwick->ahrs->north_vec.v[X] = 1.0f;
	ahrs_madgwick->ahrs->north_vec.v[Y] = 0.0f;
	ahrs_madgwick->ahrs->north_vec.v[Z] = 0.0f;
	
	ahrs_madgwick->ahrs->up_vec.s    = 0.0f;
	ahrs_madgwick->ahrs->up_vec.v[X] = 0.0f;
	ahrs_madgwick->ahrs->up_vec.v[Y] = 0.0f;
	ahrs_madgwick->ahrs->up_vec.v[Z] = -1.0f;

	// Init config
	ahrs_madgwick->beta = config->beta;
	ahrs_madgwick->zeta = config->zeta;

	return true;
}


void ahrs_madgwick_update(ahrs_madgwick_t* ahrs_madgwick)
{
	// ----
	// Variables required for computation
	// ----
	float recip_norm, norm; 													// vector norm
	float q_dot_omega_1, q_dot_omega_2, q_dot_omega_3, q_dot_omega_4; 		// quaternion rate from gyroscopes elements
	float f1, f2, f3, f4, f5, f6; 										// objective function elements
	float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; 					// objective function Jacobian elements
	float J_41, J_42, J_43, J_44, J_51, J_52, J_53, J_54, J_61, J_62, J_63, J_64; 
	float qhat_dot_1, qhat_dot_2, qhat_dot_3, qhat_dot_4; 					// estimated direction of the gyroscope error
	float w_err_x, w_err_y, w_err_z; 											// estimated direction of the gyroscope error (angular)
	float h_x, h_y, h_z;														// computed flux in the earth frame

	// Compute time since last update
	float now = time_keeper_get_s(); 
	float dt  = now - ahrs_madgwick->ahrs->last_update;

	// Get current attitude in NED
	float q1 = ahrs_madgwick->ahrs->qe.s;
	float q2 = ahrs_madgwick->ahrs->qe.v[X];
	float q3 = ahrs_madgwick->ahrs->qe.v[Y];
	float q4 = ahrs_madgwick->ahrs->qe.v[Z];

	// Get values from sensors in NED
	float wx = ahrs_madgwick->imu->scaled_gyro.data[X];  
	float wy = ahrs_madgwick->imu->scaled_gyro.data[Y];
	float wz = ahrs_madgwick->imu->scaled_gyro.data[Z];
	float ax = ahrs_madgwick->imu->scaled_accelero.data[X];
	float ay = ahrs_madgwick->imu->scaled_accelero.data[Y];
	float az = ahrs_madgwick->imu->scaled_accelero.data[Z];
	float mx = ahrs_madgwick->imu->scaled_compass.data[X];
	float my = ahrs_madgwick->imu->scaled_compass.data[Y];
	float mz = ahrs_madgwick->imu->scaled_compass.data[Z];

	// Get current estimate of earth magnetic field orientation
	float bx = ahrs_madgwick->ref_b[X];
	float bz = ahrs_madgwick->ref_b[Z];

	// Get current gyro bias estimate
	float w_bx = ahrs_madgwick->imu->calib_gyro.bias[X] * ahrs_madgwick->imu->calib_gyro.scale_factor[X];
	float w_by = ahrs_madgwick->imu->calib_gyro.bias[Y] * ahrs_madgwick->imu->calib_gyro.scale_factor[Y];
	float w_bz = ahrs_madgwick->imu->calib_gyro.bias[Z] * ahrs_madgwick->imu->calib_gyro.scale_factor[Z];

	// Compute feedback only if magnetometer measurement valid (avoids NaN in magnetometer normalisation)
	if( (mx == 0.0f) && (my == 0.0f) && (mz == 0.0f) ) 
	{
		return;
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if( (ax == 0.0f) && (ay == 0.0f) && (az == 0.0f) ) 
	{
		return;
	}
	
	// ----
	// Main computation
	// ----		
	// Auxilirary variables to avoid repeated calculations
	float half_q1 	= 0.5f * q1;
	float half_q2 	= 0.5f * q2;
	float half_q3 	= 0.5f * q3;
	float half_q4 	= 0.5f * q4;
	float two_q1 		= 2.0f * q1;
	float two_q2 		= 2.0f * q2;
	float two_q3 		= 2.0f * q3;
	float two_q4 		= 2.0f * q4;
	float two_bx 		= 2.0f * bx;
	float two_bz 		= 2.0f * bz;
	float two_bx_q1 	= 2.0f * bx * q1;
	float two_bx_q2 	= 2.0f * bx * q2;
	float two_bx_q3 	= 2.0f * bx * q3;
	float two_bx_q4 	= 2.0f * bx * q4;
	float two_bz_q1 	= 2.0f * bz * q1;
	float two_bz_q2 	= 2.0f * bz * q2;
	float two_bz_q3 	= 2.0f * bz * q3;
	float two_bz_q4 	= 2.0f * bz * q4;
	float q1_q2;
	float q1_q3 	= q1 * q3;
	float q1_q4;
	float q2_q3;
	float q2_q4 	= q2 * q4;
	float q3_q4;
	float two_mx 		= 2.0f * mx;
	float two_my 		= 2.0f * my;
	float two_mz 		= 2.0f * mz;

	// Normalise accelerometer measurement
	recip_norm = maths_fast_inv_sqrt(ax * ax + ay * ay + az * az);
	ax *= recip_norm;
	ay *= recip_norm;
	az *= recip_norm;   

	// Normalise magnetometer measurement
	recip_norm = maths_fast_inv_sqrt(mx * mx + my * my + mz * mz);
	mx *= recip_norm;
	my *= recip_norm;
	mz *= recip_norm;

	// compute the objective function and Jacobian
	f1 = -two_q2 * q4 + two_q1 * q3 - ax;
	f2 = -two_q1 * q2 - two_q3 * q4 - ay;
	f3 = -1.0f + two_q2 * q2 + two_q3 * q3 - az;
	f4 = two_bx * (0.5f - q3 * q3 - q4 * q4) + two_bz * (q2_q4 - q1_q3) - mx;
	f5 = two_bx * (q2 * q3 - q1 * q4) + two_bz * (q1 * q2 + q3 * q4) - my;
	f6 = two_bx * (q1_q3 + q2_q4) + two_bz * (0.5f - q2 * q2 - q3 * q3) - mz;
	J_11or24 = -two_q3;					// J_11 negated in matrix multiplication
	J_12or23 = -2.0f * q4;
	J_13or22 = -two_q1; 					// J_22 negated in matrix multiplication
	J_14or21 = -two_q2;
	J_32 = -2.0f * J_14or21; 				// negated in matrix multiplication
	J_33 = -2.0f * J_11or24; 				// negated in matrix multiplication
	J_41 = two_bz_q3; 					// negated in matrix multiplication
	J_42 = two_bz_q4;
	J_43 = 2.0f * two_bx_q3 + two_bz_q1; 	// negated in matrix multiplication
	J_44 = 2.0f * two_bx_q4 - two_bz_q2; 	// negated in matrix multiplication
	J_51 = two_bx_q4 - two_bz_q2; 			// negated in matrix multiplication
	J_52 = two_bx_q3 + two_bz_q1;
	J_53 = two_bx_q2 + two_bz_q4;
	J_54 = two_bx_q1 - two_bz_q3; 			// negated in matrix multiplication
	J_61 = two_bx_q3;
	J_62 = two_bx_q4 - 2.0f * two_bz_q2;
	J_63 = two_bx_q1 - 2.0f * two_bz_q3;
	J_64 = two_bx_q2;

	// compute the gradient (matrix multiplication)
	qhat_dot_1 = J_14or21 * f2 - J_11or24 * f1 - J_41 * f4 - J_51 * f5 + J_61 * f6;
	qhat_dot_2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3 + J_42 * f4 + J_52 * f5 + J_62 * f6;
	qhat_dot_3 = J_12or23 * f2 - J_33 * f3 - J_13or22 * f1 - J_43 * f4 + J_53 * f5 + J_63 * f6;
	qhat_dot_4 = J_14or21 * f1 + J_11or24 * f2 - J_44 * f4 - J_54 * f5 + J_64 * f6;

	// normalise the gradient to estimate direction of the gyroscope error
	norm = maths_fast_sqrt(qhat_dot_1 * qhat_dot_1 + qhat_dot_2 * qhat_dot_2 + qhat_dot_3 * qhat_dot_3 + qhat_dot_4 * qhat_dot_4);
	qhat_dot_1 = qhat_dot_1 / norm;
	qhat_dot_2 = qhat_dot_2 / norm;
	qhat_dot_3 = qhat_dot_3 / norm;
	qhat_dot_4 = qhat_dot_4 / norm;

	// compute angular estimated direction of the gyroscope error
	w_err_x = two_q1 * qhat_dot_2 - two_q2 * qhat_dot_1 - two_q3 * qhat_dot_4 + two_q4 * qhat_dot_3;
	w_err_y = two_q1 * qhat_dot_3 + two_q2 * qhat_dot_4 - two_q3 * qhat_dot_1 - two_q4 * qhat_dot_2;
	w_err_z = two_q1 * qhat_dot_4 - two_q2 * qhat_dot_3 + two_q3 * qhat_dot_2 - two_q4 * qhat_dot_1;

	// compute and remove the gyroscope baises
	w_bx += w_err_x * dt * ahrs_madgwick->zeta;
	w_by += w_err_y * dt * ahrs_madgwick->zeta;
	w_bz += w_err_z * dt * ahrs_madgwick->zeta;
	wx -= w_bx;
	wy -= w_by;
	wz -= w_bz;

	// compute the quaternion rate measured by gyroscopes
	q_dot_omega_1 = -half_q2 * wx - half_q3 * wy - half_q4 * wz;
	q_dot_omega_2 = half_q1 * wx + half_q3 * wz - half_q4 * wy;
	q_dot_omega_3 = half_q1 * wy - half_q2 * wz + half_q4 * wx;
	q_dot_omega_4 = half_q1 * wz + half_q2 * wy - half_q3 * wx;

	// compute then integrate the estimated quaternion rate
	q1 += (q_dot_omega_1 - (ahrs_madgwick->beta * qhat_dot_1)) * dt;
	q2 += (q_dot_omega_2 - (ahrs_madgwick->beta * qhat_dot_2)) * dt;
	q3 += (q_dot_omega_3 - (ahrs_madgwick->beta * qhat_dot_3)) * dt;
	q4 += (q_dot_omega_4 - (ahrs_madgwick->beta * qhat_dot_4)) * dt;

	// Normalise quaternion
	recip_norm = maths_fast_inv_sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
	q1 *= recip_norm;
	q2 *= recip_norm;
	q3 *= recip_norm;
	q4 *= recip_norm;

	// compute flux in the earth frame
	q1_q2 = q1 * q2; 			// recompute auxilirary variables
	q1_q3 = q1 * q3; 
	q1_q4 = q1 * q4; 
	q3_q4 = q3 * q4; 
	q2_q3 = q2 * q3; 
	q2_q4 = q2 * q4; 
	h_x = two_mx * (0.5f - q3 * q3 - q4 * q4) + two_my * (q2_q3 - q1_q4) + two_mz * (q2_q4 + q1_q3);
	h_y = two_mx * (q2_q3 + q1_q4) + two_my * (0.5f - q2 * q2 - q4 * q4) + two_mz * (q3_q4 - q1_q2);
	h_z = two_mx * (q2_q4 - q1_q3) + two_my * (q3_q4 + q1_q2) + two_mz * (0.5f - q2 * q2 - q3 * q3);

	// normalise the flux vector to have only components in the x and z
	bx = sqrt((h_x * h_x) + (h_y * h_y));
	bz = h_z;

	// ----
	// Store result in ahrs and imu structures
	// ----
	// Time
	ahrs_madgwick->ahrs->last_update 	= now;
	ahrs_madgwick->ahrs->dt 			= dt;

	// Quaternion in NED
	ahrs_madgwick->ahrs->qe.s 		= q1;
	ahrs_madgwick->ahrs->qe.v[X] 	= q2;
	ahrs_madgwick->ahrs->qe.v[Y] 	= q3;
	ahrs_madgwick->ahrs->qe.v[Z] 	= q4;

	// Estimate of earth magnetic field orientation
	ahrs_madgwick->ref_b[X] = bx;
	ahrs_madgwick->ref_b[Z] = bz;
	
	// Angular_speed.
	ahrs_madgwick->ahrs->angular_speed[X] = ahrs_madgwick->imu->scaled_gyro.data[X];
	ahrs_madgwick->ahrs->angular_speed[Y] = ahrs_madgwick->imu->scaled_gyro.data[Y];
	ahrs_madgwick->ahrs->angular_speed[Z] = ahrs_madgwick->imu->scaled_gyro.data[Z];

	// Gyro bias
	ahrs_madgwick->imu->calib_gyro.bias[X] = w_bx / ahrs_madgwick->imu->calib_gyro.scale_factor[X];
	ahrs_madgwick->imu->calib_gyro.bias[Y] = w_by / ahrs_madgwick->imu->calib_gyro.scale_factor[Y];
	ahrs_madgwick->imu->calib_gyro.bias[Z] = w_bz / ahrs_madgwick->imu->calib_gyro.scale_factor[Z];

	// Up vector
	float up_glob[3] = {0.0f, 0.0f, -1.0f}; 
	quaternions_rotate_vector(ahrs_madgwick->ahrs->qe, up_glob, ahrs_madgwick->ahrs->up_vec.v);

	// Update linear acceleration
	ahrs_madgwick->ahrs->linear_acc[X] = 9.81f * (ahrs_madgwick->imu->scaled_accelero.data[X] - ahrs_madgwick->ahrs->up_vec.v[X]) ;							// TODO: review this line!
	ahrs_madgwick->ahrs->linear_acc[Y] = 9.81f * (ahrs_madgwick->imu->scaled_accelero.data[Y] - ahrs_madgwick->ahrs->up_vec.v[Y]) ;							// TODO: review this line!
	ahrs_madgwick->ahrs->linear_acc[Z] = 9.81f * (ahrs_madgwick->imu->scaled_accelero.data[Z] - ahrs_madgwick->ahrs->up_vec.v[Z]) ;							// TODO: review this line!

	// North vector 
	// TODO: Remove as never used
	float north_glob[3] = {1.0f, 0.0f, 0.0f}; 
	quaternions_rotate_vector(ahrs_madgwick->ahrs->qe, north_glob, ahrs_madgwick->ahrs->north_vec.v);
}
