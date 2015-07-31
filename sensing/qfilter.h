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
 * \file qfilter.h
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief This file implements a complementary filter for the attitude estimation
 *
 ******************************************************************************/


#ifndef QFILTER_H_
#define QFILTER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "imu.h"
#include "ahrs.h"


/**
 * \brief The structure for configuring the quaternion-based attitude estimation
 */
typedef struct
{
	float   kp;								///< The proportional gain for the acceleration correction of the angular rates
	float   ki;								///< The integral gain for the acceleration correction of the biais
	float   kp_mag;							///< The proportional gain for the magnetometer correction of the angular rates
	float   ki_mag;							///< The integral gain for the magnetometer correction of the angular rates
} qfilter_conf_t;

/**
 * \brief The structure for the quaternion-based attitude estimation
 */
typedef struct
{
	imu_t* 	imu;			///< Pointer to inertial sensors readout
	ahrs_t* ahrs;			///< Pointer to estimated attiude
	
	float   kp;				///< The proportional gain for the acceleration correction of the angular rates
	float   ki;				///< The integral gain for the acceleration correction of the biais
	float   kp_mag;			///< The proportional gain for the magnetometer correction of the angular rates
	float   ki_mag;			///< The integral gain for the magnetometer correction of the angular rates
} qfilter_t;


/**
 * \brief	Initialize the attitude estimation module
 *
 * \param	qf				The pointer to the attitude structure
 * \param	config			The qfilter configuration gains
 * \param	imu				The pointer to the IMU structure
 * \param	ahrs			The pointer to the attitude estimation structure
 *
 * \return	True if the init succeed, false otherwise
 */
bool qfilter_init(qfilter_t* qf, const qfilter_conf_t* config, imu_t* imu, ahrs_t* ahrs);


/**
 * \brief	Performs the attitude estimation via a complementary filter
 *
 * \param	qf		The pointer to the qfilter structure
 */
void qfilter_update(qfilter_t *qf);


#ifdef __cplusplus
}
#endif

#endif /* QFILTER_H_ */
