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
 * \file bmp085.h
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief This file is the driver for the barometer module: BMP085
 * 
 ******************************************************************************/


#ifndef BMP085_H_
#define BMP085_H_

#ifdef __cplusplus
	extern "C" {
#endif

#include "scheduler.h"
#include <stdint.h>
#include <stdbool.h>
#include "barometer.h"


/**
 * \brief structure containing all the barometer's data
*/
typedef struct
{
	barometer_t* barometer;			///< Pointer to the general barometer structure
} bmp085_t;


/**
 * \brief Initialize the barometer sensor
 * 
 * \param bmp085	Pointer to the barometer 085 struct
*/
void bmp085_init(barometer_t *bmp085);


/**
 * \brief Initialize the barometer sensor in slow mode
*/
void bmp085_init_slow(void);


/**
 * \brief	Reset the altitude to position estimation origin
 *
 * \param	bmp085				Pointer to the barometer 085 struct
 * \param	origin_altitude		Altitude corresponding to the origin
 */
void bmp085_reset_origin_altitude(barometer_t* bmp085, float origin_altitude);


/**
 * \brief Update the barometer
 *
 * \param bmp085				Pointer to the barometer 085 struct
*/
void bmp085_update(barometer_t *bmp085);


#ifdef __cplusplus
}
#endif

#endif /* BMP085_H_ */