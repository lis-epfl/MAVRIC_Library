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
void ahrs_madgwick_algo(ahrs_madgwick_t* ahrs_madgwick, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void ahrs_madgwick_algo(ahrs_madgwick_t* ahrs_madgwick, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
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
		_2q0mx = 2.0f * ahrs_madgwick->q_nwu.s * mx;
		_2q0my = 2.0f * ahrs_madgwick->q_nwu.s * my;
		_2q0mz = 2.0f * ahrs_madgwick->q_nwu.s * mz;
		_2q1mx = 2.0f * ahrs_madgwick->q_nwu.v[0] * mx;
		_2q0 = 2.0f * ahrs_madgwick->q_nwu.s;
		_2q1 = 2.0f * ahrs_madgwick->q_nwu.v[0];
		_2q2 = 2.0f * ahrs_madgwick->q_nwu.v[1];
		_2q3 = 2.0f * ahrs_madgwick->q_nwu.v[2];
		_2q0q2 = 2.0f * ahrs_madgwick->q_nwu.s * ahrs_madgwick->q_nwu.v[1];
		_2q2q3 = 2.0f * ahrs_madgwick->q_nwu.v[1] * ahrs_madgwick->q_nwu.v[2];
		q0q0 = ahrs_madgwick->q_nwu.s * ahrs_madgwick->q_nwu.s;
		q0q1 = ahrs_madgwick->q_nwu.s * ahrs_madgwick->q_nwu.v[0];
		q0q2 = ahrs_madgwick->q_nwu.s * ahrs_madgwick->q_nwu.v[1];
		q0q3 = ahrs_madgwick->q_nwu.s * ahrs_madgwick->q_nwu.v[2];
		q1q1 = ahrs_madgwick->q_nwu.v[0] * ahrs_madgwick->q_nwu.v[0];
		q1q2 = ahrs_madgwick->q_nwu.v[0] * ahrs_madgwick->q_nwu.v[1];
		q1q3 = ahrs_madgwick->q_nwu.v[0] * ahrs_madgwick->q_nwu.v[2];
		q2q2 = ahrs_madgwick->q_nwu.v[1] * ahrs_madgwick->q_nwu.v[1];
		q2q3 = ahrs_madgwick->q_nwu.v[1] * ahrs_madgwick->q_nwu.v[2];
		q3q3 = ahrs_madgwick->q_nwu.v[2] * ahrs_madgwick->q_nwu.v[2];

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * ahrs_madgwick->q_nwu.v[2] + _2q0mz * ahrs_madgwick->q_nwu.v[1] + mx * q1q1 + _2q1 * my * ahrs_madgwick->q_nwu.v[1] + _2q1 * mz * ahrs_madgwick->q_nwu.v[2] - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * ahrs_madgwick->q_nwu.v[2] + my * q0q0 - _2q0mz * ahrs_madgwick->q_nwu.v[0] + _2q1mx * ahrs_madgwick->q_nwu.v[1] - my * q1q1 + my * q2q2 + _2q2 * mz * ahrs_madgwick->q_nwu.v[2] - my * q3q3;
		_2bx = maths_fast_sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * ahrs_madgwick->q_nwu.v[1] + _2q0my * ahrs_madgwick->q_nwu.v[0] + mz * q0q0 + _2q1mx * ahrs_madgwick->q_nwu.v[2] - mz * q1q1 + _2q2 * my * ahrs_madgwick->q_nwu.v[2] - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * ahrs_madgwick->q_nwu.v[1] * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * ahrs_madgwick->q_nwu.v[2] + _2bz * ahrs_madgwick->q_nwu.v[0]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * ahrs_madgwick->q_nwu.v[1] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * ahrs_madgwick->q_nwu.v[0] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * ahrs_madgwick->q_nwu.v[2] * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * ahrs_madgwick->q_nwu.v[1] + _2bz * ahrs_madgwick->q_nwu.s) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * ahrs_madgwick->q_nwu.v[2] - _4bz * ahrs_madgwick->q_nwu.v[0]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * ahrs_madgwick->q_nwu.v[1] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * ahrs_madgwick->q_nwu.v[1] - _2bz * ahrs_madgwick->q_nwu.s) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * ahrs_madgwick->q_nwu.v[0] + _2bz * ahrs_madgwick->q_nwu.v[2]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * ahrs_madgwick->q_nwu.s - _4bz * ahrs_madgwick->q_nwu.v[1]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * ahrs_madgwick->q_nwu.v[2] + _2bz * ahrs_madgwick->q_nwu.v[0]) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * ahrs_madgwick->q_nwu.s + _2bz * ahrs_madgwick->q_nwu.v[1]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * ahrs_madgwick->q_nwu.v[0] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = maths_fast_inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// compute gyro drift bias
		_w_err_x = _2q0 * s1 - _2q1 * s0 - _2q2 * s3 + _2q3 * s2;
		_w_err_y = _2q0 * s2 + _2q1 * s3 - _2q2 * s0 - _2q3 * s1;
		_w_err_z = _2q0 * s3 - _2q1 * s2 + _2q2 * s1 - _2q3 * s0;
		
		ahrs_madgwick->w_bx += _w_err_x * ahrs_madgwick->ahrs->dt * ahrs_madgwick->zeta;
		ahrs_madgwick->w_by += _w_err_y * ahrs_madgwick->ahrs->dt * ahrs_madgwick->zeta;
		ahrs_madgwick->w_bz += _w_err_z * ahrs_madgwick->ahrs->dt * ahrs_madgwick->zeta;
		
		gx -= ahrs_madgwick->w_bx;
		gy -= ahrs_madgwick->w_by;
		gz -= ahrs_madgwick->w_bz;
		
		// Rate of change of quaternion from gyroscope
		qDot1 = 0.5f * (-ahrs_madgwick->q_nwu.v[0] * gx - ahrs_madgwick->q_nwu.v[1] * gy - ahrs_madgwick->q_nwu.v[2] * gz);
		qDot2 = 0.5f * ( ahrs_madgwick->q_nwu.s * gx + ahrs_madgwick->q_nwu.v[1] * gz - ahrs_madgwick->q_nwu.v[2] * gy);
		qDot3 = 0.5f * ( ahrs_madgwick->q_nwu.s * gy - ahrs_madgwick->q_nwu.v[0] * gz + ahrs_madgwick->q_nwu.v[2] * gx);
		qDot4 = 0.5f * ( ahrs_madgwick->q_nwu.s * gz + ahrs_madgwick->q_nwu.v[0] * gy - ahrs_madgwick->q_nwu.v[1] * gx);

		// Apply feedback step
		qDot1 -= ahrs_madgwick->beta * s0;
		qDot2 -= ahrs_madgwick->beta * s1;
		qDot3 -= ahrs_madgwick->beta * s2;
		qDot4 -= ahrs_madgwick->beta * s3;
	}
	else
	{
		// Rate of change of quaternion from gyroscope
		qDot1 = 0.5f * (-ahrs_madgwick->q_nwu.v[0] * gx - ahrs_madgwick->q_nwu.v[1] * gy - ahrs_madgwick->q_nwu.v[2] * gz);
		qDot2 = 0.5f * ( ahrs_madgwick->q_nwu.s * gx + ahrs_madgwick->q_nwu.v[1] * gz - ahrs_madgwick->q_nwu.v[2] * gy);
		qDot3 = 0.5f * ( ahrs_madgwick->q_nwu.s * gy - ahrs_madgwick->q_nwu.v[0] * gz + ahrs_madgwick->q_nwu.v[2] * gx);
		qDot4 = 0.5f * ( ahrs_madgwick->q_nwu.s * gz + ahrs_madgwick->q_nwu.v[0] * gy - ahrs_madgwick->q_nwu.v[1] * gx);
	}

	// Integrate rate of change of quaternion to yield quaternion
	ahrs_madgwick->q_nwu.s += qDot1 * ahrs_madgwick->ahrs->dt;
	ahrs_madgwick->q_nwu.v[0] += qDot2 * ahrs_madgwick->ahrs->dt;
	ahrs_madgwick->q_nwu.v[1] += qDot3 * ahrs_madgwick->ahrs->dt;
	ahrs_madgwick->q_nwu.v[2] += qDot4 * ahrs_madgwick->ahrs->dt;

	// Normalise quaternion
	recipNorm = maths_fast_inv_sqrt(ahrs_madgwick->q_nwu.s * ahrs_madgwick->q_nwu.s + ahrs_madgwick->q_nwu.v[0] * ahrs_madgwick->q_nwu.v[0] + ahrs_madgwick->q_nwu.v[1] * ahrs_madgwick->q_nwu.v[1] + ahrs_madgwick->q_nwu.v[2] * ahrs_madgwick->q_nwu.v[2]);
	ahrs_madgwick->q_nwu.s *= recipNorm;
	ahrs_madgwick->q_nwu.v[0] *= recipNorm;
	ahrs_madgwick->q_nwu.v[1] *= recipNorm;
	ahrs_madgwick->q_nwu.v[2] *= recipNorm;
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------


bool ahrs_madgwick_init(ahrs_madgwick_t* ahrs_madgwick, const ahrs_madgwick_conf_t* config, imu_t* imu, ahrs_t* ahrs, const position_estimation_t* pos_est)
{
	// Init dependencies
	ahrs_madgwick->imu 	= imu;
	ahrs_madgwick->ahrs = ahrs;
	ahrs_madgwick->pos_est = pos_est;
	
	// Init config
	ahrs_madgwick->beta = config->beta;
	ahrs_madgwick->zeta = config->zeta;
	ahrs_madgwick->acceleration_correction = config->acceleration_correction;
	ahrs_madgwick->correction_speed = config->correction_speed;
	
	// Init global variables
	ahrs_madgwick->q_nwu.s = 1.0f;
	ahrs_madgwick->q_nwu.v[0] = 0.0f;
	ahrs_madgwick->q_nwu.v[1] = 0.0f;
	ahrs_madgwick->q_nwu.v[2] = 0.0f;
	ahrs_madgwick->w_bx = 0.0f;
	ahrs_madgwick->w_by = 0.0f;
	ahrs_madgwick->w_bz = 0.0f;
	
	// Set flag
	ahrs_madgwick->ahrs->internal_state = AHRS_READY;

	ahrs_madgwick->last_airspeed = 0.0f;

	// Notify success
	print_util_dbg_print("[AHRS MADGWICK] Initialised.\r\n");

	return true;
}


void ahrs_madgwick_update(ahrs_madgwick_t* ahrs_madgwick)
{
	// Compute time
	uint32_t t = time_keeper_get_time_ticks();			// Warning: time is given in uint32_t here !
	float dt = time_keeper_ticks_to_seconds(t - ahrs_madgwick->ahrs->last_update);
	
		// Time
	ahrs_madgwick->ahrs->last_update 	= t;
	ahrs_madgwick->ahrs->dt 			= dt;
	
	////////////////////////////////////////////////////
	// Compute correction for parasitic accelerations //
	////////////////////////////////////////////////////
	// Based on Adrien Briod report about EKF for Quaternion-based orientation estimation, using speed and inertial sensors
	// Centrifugal force
	float hc_y =  - (ahrs_madgwick->pos_est->vel_bf[X] * ahrs_madgwick->imu->scaled_gyro.data[Z]);
	float hc_z = ahrs_madgwick->pos_est->vel_bf[X] * ahrs_madgwick->imu->scaled_gyro.data[Y];
	
	// Longitudinal accelerations
	float hdv_x = - (ahrs_madgwick->pos_est->vel_bf[X] - ahrs_madgwick->last_airspeed)/dt;
	
	
	/////////////////////////////////////////////////////////////
	// Get measurements with correction, in the MADGWICK FRAME //
	/////////////////////////////////////////////////////////////
	// Transform sensor measurements
	float gx =   ahrs_madgwick->imu->scaled_gyro.data[X];
	float gy = - ahrs_madgwick->imu->scaled_gyro.data[Y];
	float gz = - ahrs_madgwick->imu->scaled_gyro.data[Z];
	float ax =   ahrs_madgwick->imu->scaled_accelero.data[X];
	float ay = - ahrs_madgwick->imu->scaled_accelero.data[Y];
	float az = - ahrs_madgwick->imu->scaled_accelero.data[Z];
	if(ahrs_madgwick->acceleration_correction == 1 && ahrs_madgwick->pos_est->vel_bf[X] >= ahrs_madgwick->correction_speed)
	{
		ax =   (ahrs_madgwick->imu->scaled_accelero.data[X] + hdv_x);
		ay = - (ahrs_madgwick->imu->scaled_accelero.data[Y] + hc_y);
		az = - (ahrs_madgwick->imu->scaled_accelero.data[Z] + hc_z);
	}
	float mx =   ahrs_madgwick->imu->scaled_compass.data[X];
	float my = - ahrs_madgwick->imu->scaled_compass.data[Y];
	float mz = - ahrs_madgwick->imu->scaled_compass.data[Z];

	
	//////////////////////////////////////////////
	// Run the algorithm, in the MADGWICK FRAME //
	//////////////////////////////////////////////
	ahrs_madgwick_algo( ahrs_madgwick, 
						gx, gy, gz,
						ax, ay, az,
						mx, my, mz);
	
	
	/////////////////////////////////////////////////////////////////
	// Compute values in MAVRIC FRAME and write back into IMU/AHRS //
	/////////////////////////////////////////////////////////////////
	// Quaternion rotation to express the result in the MAVRIC FRAME
	quat_t q_nwu2ned = {.s=0, .v={1.0f, 0.0f, 0.0f}};		// Quaternion used for transformation
	quat_t q_ned = {.s=ahrs_madgwick->q_nwu.s, .v={0.0f, 0.0f, 0.0f}};			// Current quaternion in mavric frame
	
	quat_t qinv, qtmp;										// Transformation q_rot^(-1) * q * q_rot
	qinv = quaternions_inverse(q_nwu2ned);
	qtmp = quaternions_multiply(qinv,ahrs_madgwick->q_nwu);
	q_ned = quaternions_multiply(qtmp,q_nwu2ned);

	// Quaternion in NED
	ahrs_madgwick->ahrs->qe = q_ned;
	
	// Angular_speed, subtract estimated biases, making the correspondance between frames !
	ahrs_madgwick->ahrs->angular_speed[X] = ahrs_madgwick->imu->scaled_gyro.data[X] - (ahrs_madgwick->w_bx);
	ahrs_madgwick->ahrs->angular_speed[Y] = ahrs_madgwick->imu->scaled_gyro.data[Y] - (-ahrs_madgwick->w_by);
	ahrs_madgwick->ahrs->angular_speed[Z] = ahrs_madgwick->imu->scaled_gyro.data[Z] - (-ahrs_madgwick->w_bz);
	// Update angular speeds and update IMU biases !
	//ahrs_madgwick->ahrs->angular_speed[X] = ahrs_madgwick->imu->scaled_gyro.data[X];
	//ahrs_madgwick->ahrs->angular_speed[Y] = ahrs_madgwick->imu->scaled_gyro.data[Y];
	//ahrs_madgwick->ahrs->angular_speed[Z] = ahrs_madgwick->imu->scaled_gyro.data[Z];
	//ahrs_madgwick->imu->calib_gyro.bias[X] = ahrs_madgwick->w_bx/ahrs_madgwick->imu->calib_gyro.scale_factor[X];
	//ahrs_madgwick->imu->calib_gyro.bias[Y] = -ahrs_madgwick->w_by/ahrs_madgwick->imu->calib_gyro.scale_factor[Y];
	//ahrs_madgwick->imu->calib_gyro.bias[Z] = -ahrs_madgwick->w_bz/ahrs_madgwick->imu->calib_gyro.scale_factor[Z];

	// Up vector
	float up_loc[3] = {0.0f, 0.0f, -1.0f}; 
	quaternions_rotate_vector(quaternions_inverse(ahrs_madgwick->ahrs->qe), up_loc, ahrs_madgwick->ahrs->up_vec.v);

	// Update linear acceleration
	ahrs_madgwick->ahrs->linear_acc[X] = 9.81f * (ahrs_madgwick->imu->scaled_accelero.data[X] - ahrs_madgwick->ahrs->up_vec.v[X]) ;							// TODO: review this line!
	ahrs_madgwick->ahrs->linear_acc[Y] = 9.81f * (ahrs_madgwick->imu->scaled_accelero.data[Y] - ahrs_madgwick->ahrs->up_vec.v[Y]) ;							// TODO: review this line!
	ahrs_madgwick->ahrs->linear_acc[Z] = 9.81f * (ahrs_madgwick->imu->scaled_accelero.data[Z] - ahrs_madgwick->ahrs->up_vec.v[Z]) ;							// TODO: review this line!

	// North vector 
	// TODO: Remove as never used
	float north_loc[3] = {1.0f, 0.0f, 0.0f}; 
	quaternions_rotate_vector(quaternions_inverse(ahrs_madgwick->ahrs->qe), north_loc, ahrs_madgwick->ahrs->north_vec.v);
}

