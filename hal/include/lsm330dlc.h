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
 * \file lsm330dlc.h
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Geraud L'Eplattenier
 *   
 * \brief This file is the driver for the integrated 3axis gyroscope and 
 * accelerometer LSM330DLC
 * 
 ******************************************************************************/


#ifndef LSM330DLC_H_
#define LSM330DLC_H_

#ifdef __cplusplus
	extern "C" {
#endif

#include <stdint.h>
#include "gyroscope.h"
#include "accelerometer.h"		

/**
 * \brief	Structure containing the accelerometer's data
*/
typedef struct
{
	accelerometer_t *raw_accelero;
} lsm_acc_t;

/**
 * \brief	Structure containing the gyroscope's data
*/
typedef struct
{
	gyroscope_t *raw_gyro;
} lsm_gyro_t;


/**
 * \brief	Initialize the LSM330 accelerometer+gyroscope sensor
*/
void lsm330dlc_init(void);

/**
 * \brief	Return the gyroscope's data
 *
 * \param	lsm_gyro_outputs	Pointer to the gyroscope data structure
*/
void lsm330dlc_gyro_update(gyroscope_t *lsm_gyro_outputs);

/**
 * \brief	Return the accelerometer's data
 *
 * \param	lsm_acc_outputs		Pointer to the accelerometer data structure
*/
void lsm330dlc_acc_update(accelerometer_t *lsm_acc_outputs);

#ifdef __cplusplus
	}
#endif

#endif 