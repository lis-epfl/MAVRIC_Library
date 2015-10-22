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
 * \file ahrs_madgwick.c
 * 
 * \author MAV'RIC Team
 * \author SOH Madgwick
 * \author Julien Lecoeur, Simon Pyroth
 *   
 * \brief Implementation of Madgwick's AHRS algorithms.
 *
 * See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
 * And: https://github.com/ccny-ros-pkg/imu_tools/blob/indigo/imu_filter_madgwick/src/imu_filter.cpp
 *
 * Date			Author          Notes
 * 29/09/2011	SOH Madgwick    Initial release
 * 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
 * 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
 * 04/02/2014	Julien Lecoeur	Adapt to MAVRIC
 * 22/10/2015	Simon Pyroth	Updated version
 *
 ******************************************************************************/


/**
 *   Disclaimer: this WIP
 */



#include "ahrs_madgwick.h"
#include "maths.h"
#include "print_util.h"
#include "constants.h"
#include "time_keeper.h"
#include "quaternions.h"



//------------------------------------------------------------------------------
// PRIVATE VARIABLES DECLARATION
//------------------------------------------------------------------------------

// Global variables used in the Madgwick algorithm.
// Quaternions used for the Madgwick algorithm. They are in the MADGWICK FRAME, not in the MAVRIC FRAME !
float q0, q1, q2, q3;

// Estimated gyro biases. Warning: they are also expressed in MADGWICK FRAME ! Transformation has to be made to express them in MAVRIC FRAME !
float w_bx_, w_by_, w_bz_;


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief  	Perform Madgwick algorithm. All parameters (gyro, accelero, magneto, quaternions, biases are expresses in MADGWICK FRAME).
 * 
 * \param 	gx		 	Gyro x
 * \param 	gx		 	Gyro y
 * \param 	gx		 	Gyro z
 * \param 	gx		 	Accelero x
 * \param 	gx		 	Accelero y
 * \param 	gx		 	Accelero z
 * \param 	gx		 	Magneto x
 * \param 	gx		 	Magneto y
 * \param 	gx		 	Magneto z
 * \param 	beta		Beta gain
 * \param 	zeta		Zeta gain
 * \param 	dt		 	Time increment for integration
 */
void ahrs_madgwick_algo(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float beta, float zeta, float dt);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void ahrs_madgwick_algo(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float beta, float zeta, float dt)
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float _w_err_x, _w_err_y, _w_err_z;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
	{
		//madgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az, dt);		// TODO: Implement this function using link in brief to run algo even without magnetometer
		return;
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
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
		_2bx = maths_fast_sqrt(hx * hx + hy * hy);
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

		// compute gyro drift bias
		_w_err_x = _2q0 * s1 - _2q1 * s0 - _2q2 * s3 + _2q3 * s2;
		_w_err_y = _2q0 * s2 + _2q1 * s3 - _2q2 * s0 - _2q3 * s1;
		_w_err_z = _2q0 * s3 - _2q1 * s2 + _2q2 * s1 - _2q3 * s0;
		
		w_bx_ += _w_err_x * dt * zeta;
		w_by_ += _w_err_y * dt * zeta;
		w_bz_ += _w_err_z * dt * zeta;
		
		gx -= w_bx_;
		gy -= w_by_;
		gz -= w_bz_;
		
		// Rate of change of quaternion from gyroscope
		qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
		qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
		qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
		qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}
	else
	{
		// Rate of change of quaternion from gyroscope
		qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
		qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
		qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
		qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);
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
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------


bool ahrs_madgwick_init(ahrs_madgwick_t* ahrs_madgwick, const ahrs_madgwick_conf_t* config, imu_t* imu, ahrs_t* ahrs)
{
	// Init dependencies
	ahrs_madgwick->imu 	= imu;
	ahrs_madgwick->ahrs = ahrs;
	
	// Init config
	ahrs_madgwick->beta = config->beta;
	ahrs_madgwick->zeta = config->zeta;
	
	// Init global variables
	q0 = 1.0f;
	q1 = 0.0f;
	q2 = 0.0f;
	q3 = 0.0f;
	w_bx_ = 0.0f;
	w_by_ = 0.0f;
	w_bz_ = 0.0f;

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

	// Notify success
	print_util_dbg_print("[AHRS MADGWICK] Initialised.\r\n");

	return true;
}


void ahrs_madgwick_update(ahrs_madgwick_t* ahrs_madgwick)
{
	/////////////////////////////////////////////
	// Get measurements, in the MADGWICK FRAME //
	/////////////////////////////////////////////
	// Compute time
	uint32_t t = time_keeper_get_time_ticks();			// Warning: time is given in uint32_t here !
	float dt = time_keeper_ticks_to_seconds(t - ahrs_madgwick->ahrs->last_update);
	
	// Transform sensor measurements
	float gx =  ahrs_madgwick->imu->scaled_gyro.data[X];
	float gy = -ahrs_madgwick->imu->scaled_gyro.data[Y];
	float gz = -ahrs_madgwick->imu->scaled_gyro.data[Z];
	float ax =  ahrs_madgwick->imu->scaled_accelero.data[X];
	float ay = -ahrs_madgwick->imu->scaled_accelero.data[Y];
	float az = -ahrs_madgwick->imu->scaled_accelero.data[Z];
	float mx =  ahrs_madgwick->imu->scaled_compass.data[X];
	float my = -ahrs_madgwick->imu->scaled_compass.data[Y];
	float mz = -ahrs_madgwick->imu->scaled_compass.data[Z];

	
	//////////////////////////////////////////////
	// Run the algorithm, in the MADGWICK FRAME //
	//////////////////////////////////////////////
	ahrs_madgwick_algo( gx, gy, gz,
						ax, ay, az,
						mx, my, mz,
						ahrs_madgwick->beta, ahrs_madgwick->zeta, dt);
	
	
	/////////////////////////////////////////////////////////////////
	// Compute values in MAVRIC FRAME and write back into IMU/AHRS //
	/////////////////////////////////////////////////////////////////
	// Quaternion rotation to express the result in the MAVRIC FRAME
	quat_t q_nwu = {.s=q0, .v={q1, q2, q3}};				// Current quaternion in madgwick frame
	quat_t q_nwu2ned = {.s=0, .v={1.0f, 0.0f, 0.0f}};		// Quaternion used for transformation
	quat_t q_ned = {.s=q0, .v={0.0f, 0.0f, 0.0f}};			// Current quaternion in mavric frame
	
	quat_t qinv, qtmp;										// Transformation q_rot^(-1) * q * q_rot
	qinv = quaternions_inverse(q_nwu2ned);
	qtmp = quaternions_multiply(qinv,q_nwu);
	q_ned = quaternions_multiply(qtmp,q_nwu2ned);
	
	// Time
	ahrs_madgwick->ahrs->last_update 	= t;
	ahrs_madgwick->ahrs->dt 			= dt;

	// Quaternion in NED
	ahrs_madgwick->ahrs->qe = q_ned;
	
	// Angular_speed, subtract estimated biases, making the correspondance between frames !
	ahrs_madgwick->ahrs->angular_speed[X] = ahrs_madgwick->imu->scaled_gyro.data[X] - (w_bx_);
	ahrs_madgwick->ahrs->angular_speed[Y] = ahrs_madgwick->imu->scaled_gyro.data[Y] - (-w_by_);
	ahrs_madgwick->ahrs->angular_speed[Z] = ahrs_madgwick->imu->scaled_gyro.data[Z] - (-w_bz_);

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

