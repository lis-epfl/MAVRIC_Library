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
 * \file itg3200.h
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief This file is the driver for the integrated triple axis gyroscope ITG3200
 *
 ******************************************************************************/


#ifndef ITG3200_H_
#define ITG3200_H_

#ifdef __cplusplus
	extern "C" {
#endif

#include <stdint.h>

#define GY_X 0									///< Define the X Axis of the Gyroscope, as the first one of the gyroData array
#define GY_Y 1									///< Define the Y Axis of the Gyroscope, as the second one of the gyroData array
#define GY_Z 2									///< Define the Z Axis of the Gyroscope, as the third one of the gyroData array

#define ITG3200_SLAVE_ADDRESS 0b01101000		///< Define the Address of the gyroscope as a slave i2c device

/**
 * \brief structure for the gyroscope's data
*/
typedef struct
{
	int16_t temperature;	///< temperature measured(?) by the gyroscope
	int16_t axes[3];		///< buffer to store the 3 axis rates for the gyroscope
} gyroscope_t; 


/**
 * \brief Initialize the gyroscope sensor
 * not implemented yet
*/
void itg3200_init(void);

/**
 * \brief Reconfigure the gyroscope sensor
*/
void itg3200_reconfigure_gyro(void);

/**
 * \brief Return the gyroscope's data in normal mode
 * not implemented yet
 *
 * \return a pointer to the gyroscope data
*/
gyroscope_t* itg3200_get_gyro_data(void);

/**
 * \brief Initialize the gyroscope sensor in slow mode
*/
void itg3200_init_slow(void);

/**
 * \brief Return the gyroscope's data in slow mode
 *
 * \return a pointer to the gyroscope data
*/
gyroscope_t* itg3200_get_data_slow(void);

#ifdef __cplusplus
	}
#endif

#endif /* ITG3200_H_ */