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
 * \file ahrs_madgwick.h
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


#include "ahrs_madgwick.h"
#include "maths.h"
#include "print_util.h"
#include "constants.h"
#include "time_keeper.h"
#include "quaternions.h"


bool ahrs_madgwick_init(ahrs_madgwick_t* ahrs_madgwick, const ahrs_madgwick_conf_t* config, const imu_t* imu, ahrs_t* ahrs)
{
	// Init dependencies
	ahrs_madgwick->imu 	= imu;
	ahrs_madgwick->ahrs = ahrs;

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
	


	// Notify success
	print_util_dbg_print("[AHRS MADGWICK] Initialised.\r\n");

	return true;
}


void ahrs_madgwick_update(ahrs_madgwick_t* ahrs_madgwick)
{
	// Variables required for computation
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	//quat_t q_enu;
	//quat_t q_enu2ned = {.s=0, .v={0.7071f, 0.7071f, 0.0f}};
	quat_t q_nwu;
	quat_t q_nwu2ned = {.s=0, .v={1.0f, 0.0f, 0.0f}};

	// // Get current attitude in ENU
	// q_enu = quaternions_rotate(ahrs_madgwick->ahrs->qe, quaternions_inverse(q_enu2ned));
	// float q0 = q_enu.s;
	// float q1 = q_enu.v[X];
	// float q2 = q_enu.v[Y];
	// float q3 = q_enu.v[Z];

	// // Get values from sensors in ENU
	// float gy = ahrs_madgwick->imu->scaled_gyro.data[X];  
	// float gx = ahrs_madgwick->imu->scaled_gyro.data[Y];
	// float gz = -ahrs_madgwick->imu->scaled_gyro.data[Z];
	// float ay = ahrs_madgwick->imu->scaled_accelero.data[X];
	// float ax = ahrs_madgwick->imu->scaled_accelero.data[Y];
	// float az = -ahrs_madgwick->imu->scaled_accelero.data[Z];
	// float my = ahrs_madgwick->imu->scaled_compass.data[X];
	// float mx = ahrs_madgwick->imu->scaled_compass.data[Y];
	// float mz = -ahrs_madgwick->imu->scaled_compass.data[Z];

	// Get current attitude in NWU
	q_nwu = quaternions_rotate(ahrs_madgwick->ahrs->qe, quaternions_inverse(q_nwu2ned));
	float q0 = q_nwu.s;
	float q1 = q_nwu.v[X];
	float q2 = q_nwu.v[Y];
	float q3 = q_nwu.v[Z];

	// Get values from sensors in NWU
	float gx = ahrs_madgwick->imu->scaled_gyro.data[X];  
	float gy = -ahrs_madgwick->imu->scaled_gyro.data[Y];
	float gz = -ahrs_madgwick->imu->scaled_gyro.data[Z];
	float ax = ahrs_madgwick->imu->scaled_accelero.data[X];
	float ay = -ahrs_madgwick->imu->scaled_accelero.data[Y];
	float az = -ahrs_madgwick->imu->scaled_accelero.data[Z];
	// float mx = ahrs_madgwick->imu->scaled_compass.data[X];
	// float my = -ahrs_madgwick->imu->scaled_compass.data[Y];
	// float mz = -ahrs_madgwick->imu->scaled_compass.data[Z];
	float mx = ahrs_madgwick->imu->scaled_compass.data[X];
	float my = -ahrs_madgwick->imu->scaled_compass.data[Y];
	float mz = -ahrs_madgwick->imu->scaled_compass.data[Z];

	// Compute time since last update
	float now 	= time_keeper_get_time(); 
	float dt 	= now - ahrs_madgwick->ahrs->last_update;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if( !((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) ) 
	{
		// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		if( !((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)) ) 
		{
			// Normalise accelerometer measurement
			recipNorm = maths_fast_inv_sqrt(ax * ax + ay * ay + az * az);
			ax *= recipNorm;
			ay *= recipNorm;
			az *= recipNorm;   

			// Normalise magnetometer measurement
			recipNorm = maths_fast_inv_sqrt(mx * mx + my * my + mz * mz);
			mx *= recipNorm;
			my *= recipNorm;
			mz *= recipNorm;

			// Auxiliary variables to avoid repeated arithmetic
			_2q0mx = 2.0f * q0 * mx;
			_2q0my = 2.0f * q0 * my;
			_2q0mz = 2.0f * q0 * mz;
			_2q1mx = 2.0f * q1 * mx;
			_2q0 = 2.0f * q0;
			_2q1 = 2.0f * q1;
			_2q2 = 2.0f * q2;
			_2q3 = 2.0f * q3;
			_2q0q2 = 2.0f * q0 * q2;
			_2q2q3 = 2.0f * q2 * q3;
			q0q0 = q0 * q0;
			q0q1 = q0 * q1;
			q0q2 = q0 * q2;
			q0q3 = q0 * q3;
			q1q1 = q1 * q1;
			q1q2 = q1 * q2;
			q1q3 = q1 * q3;
			q2q2 = q2 * q2;
			q2q3 = q2 * q3;
			q3q3 = q3 * q3;

			// Reference direction of Earth's magnetic field
			hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
			hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
			_2bx = sqrt(hx * hx + hy * hy);
			_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
			_4bx = 2.0f * _2bx;
			_4bz = 2.0f * _2bz;

			// Gradient decent algorithm corrective step
			s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			recipNorm = maths_fast_inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
			s0 *= recipNorm;
			s1 *= recipNorm;
			s2 *= recipNorm;
			s3 *= recipNorm;

			// Apply feedback step
			qDot1 -= ahrs_madgwick->beta * s0;
			qDot2 -= ahrs_madgwick->beta * s1;
			qDot3 -= ahrs_madgwick->beta * s2;
			qDot4 -= ahrs_madgwick->beta * s3;				
		}
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * dt;
	q1 += qDot2 * dt;
	q2 += qDot3 * dt;
	q3 += qDot4 * dt;

	// Normalise quaternion
	recipNorm = maths_fast_inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	// Store result in ahrs structure
	// Time
	ahrs_madgwick->ahrs->last_update 	= now;
	ahrs_madgwick->ahrs->dt 			= dt;

	// Quaternion in NED
	// q_enu.s 	= q0;
	// q_enu.v[X] = q1;
	// q_enu.v[Y] = q2;
	// q_enu.v[Z] = q3;
	// ahrs_madgwick->ahrs->qe = quaternions_rotate(q_enu, q_enu2ned);

	q_nwu.s 	= q0;
	q_nwu.v[X] = q1;
	q_nwu.v[Y] = q2;
	q_nwu.v[Z] = q3;
	ahrs_madgwick->ahrs->qe = quaternions_rotate(q_nwu, q_nwu2ned);

	// Angular_speed.
	ahrs_madgwick->ahrs->angular_speed[X] = ahrs_madgwick->imu->scaled_gyro.data[X];
	ahrs_madgwick->ahrs->angular_speed[Y] = ahrs_madgwick->imu->scaled_gyro.data[Y];
	ahrs_madgwick->ahrs->angular_speed[Z] = ahrs_madgwick->imu->scaled_gyro.data[Z];

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