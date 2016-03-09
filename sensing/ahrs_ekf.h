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
 * \file ahrs_ekf.h
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *   
 * \brief Extended Kalman Filter attitude estimation, mixing accelerometer and magnetometer
 *
 ******************************************************************************/

#ifndef __AHRS_EKF_HPP__
#define __AHRS_EKF_HPP__

#ifdef __cplusplus
extern "C" {
#endif

#include "imu.h"
#include "ahrs.h"

/*
 * x[0] : bias_x
 * x[1] : bias_y
 * x[2] : bias_z
 * x[3] : q0
 * x[4] : q1
 * x[5] : q2
 * x[6] : q3

*/

typedef struct
{
	float sigma_w_sqr;									///< The square of the variance on the gyro bias
	float sigma_r_sqr;									///< The square of the variance on the quaternion

	float acc_norm_noise;								///< The noise gain depending on the norm of the acceleration
	float acc_multi_noise;								///< The multiplication factor in the computation of the noise for the accelerometer

	float R_acc;										///< The variance of the accelerometer
	float R_mag;										///< The variance of the magnetometer
}ahrs_ekf_config_t;

typedef struct
{
	float x[7];											///< The state of the EKF

	ahrs_t* ahrs;										///< The pointer to the ahrs structure
	imu_t* imu;											///< The pointer to the IMU structure

	bool north_calib_started;							///< The flag to calibrate the north vector at the end of the procedure

	ahrs_ekf_config_t config;							///< The config structure for the EKF module
}ahrs_ekf_t;


/**
 * \brief	Init AHRS EKF controller
 *
 * \param	ahrs_ekf 		The pointer to the AHRS EKF structure
 * \param	config 			The pointer to the ahrs_ekf configuration structure
 * \param 	imu 			The pointer to the IMU structure
 * \param 	ahrs 			The pointer to the AHRS structure
 * 
 * \return 	success
 */
bool ahrs_ekf_init(ahrs_ekf_t* ahrs_ekf, const ahrs_ekf_config_t* config, imu_t* imu, ahrs_t* ahrs);

/**
 * \brief	Performs the EKF algorithm
 *
 * \param	ahrs_ekf		The pointer to the AHRS EKF structure
 *
 * \return	true if success
 */
bool ahrs_ekf_update(ahrs_ekf_t* ahrs_ekf);

#ifdef __cplusplus
}
#endif

#endif // __AHRS_EKF_HPP__