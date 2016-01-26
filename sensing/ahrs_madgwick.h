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
 * \author Julien Lecoeur, SImon Pyroth
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


#ifndef AHRS_MADGWICK_H_
#define AHRS_MADGWICK_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "ahrs.h"
#include "imu.h"


/**
 * \brief 	Configuration for ahrs _madgwick
 */
typedef struct
{
	float 	beta;						// 2 * proportional gain (Kp)
	float	zeta;						// Gyro drift bias gain
	bool acceleration_correction;		// Enable the correction of the parasitic accelerations ?
	float correction_speed;				// Airspeed from which the correction should start
} ahrs_madgwick_conf_t;


/**
 * \brief 	Structure for the Madgwick attitude estimation filter
 */
typedef struct
{
	imu_t* 	imu;							// Pointer to IMU sensors
	ahrs_t* ahrs;							// Estimated attitude
	float 	beta;							// 2 * proportional gain (Kp)
	float	zeta;							// Gyro drift bias gain
	uint32_t acceleration_correction;		// Enable the correction of the parasitic accelerations ?
	float correction_speed;					// Airspeed from which the correction should start
} ahrs_madgwick_t;


/**
 * \brief  	Init function. This function has to be called after ahrs init to overwrite flags
 * 
 * \param 	ahrs_madgwick 	Pointer to data structure
 * \param 	config 			Pointer to config structure
 * \param 	ahrs 			Pointer to AHRS structure
 * \param 	imu 			Pointer to IMU structure
 * 
 * \return 	True if success, false if not
 */
bool ahrs_madgwick_init(ahrs_madgwick_t* ahrs_madgwick, const ahrs_madgwick_conf_t* config, imu_t* imu, ahrs_t* ahrs);


/**
 * \brief 	Main update function. Run Madgwick algorithm in its own frame and convert the result in MAVRIC frame to write the in the IMU.
 * 
 * \param 	ahrs_madgwick 	Pointer to data structure
 */
void ahrs_madgwick_update(ahrs_madgwick_t* ahrs_madgwick);


#ifdef __cplusplus
}
#endif

#endif /* AHRS_MADGWICK_H_ */