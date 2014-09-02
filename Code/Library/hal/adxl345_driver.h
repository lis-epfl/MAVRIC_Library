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
 * \file adxl345_driver.h
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief This file is the Driver for the ADXL345 accelerometer 
 *
 ******************************************************************************/


#ifndef ADXL345_DRIVER_H_
#define ADXL345_DRIVER_H_

#ifdef __cplusplus
	extern "C" {
#endif

#include <stdint.h>

#define ACC_X 0		///< number of the X axis, to be used in an array
#define ACC_Y 1		///< number of the Y axis, to be used in an array
#define ACC_Z 2		///< number of the Z axis, to be used in an array


#define ADXL_ALT_SLAVE_ADDRESS 0x53  ///< address of adxl345, as a slave on the i2c bus

/**
 * \brief structure containing the raw values + values on the 3 axis for the accelerometer
*/
typedef struct{
	uint8_t raw_data[6];	///< raw data for the 3 axis are stored in 2 uint8_t
	int16_t axes[3];		///< value of the accelerometer on each axis as a combination of 2 uint8_t
} acc_data_t; 

/**
 * \brief Initializes the accelerometer
*/
void adxl345_driver_init(void);

/**
 * \brief Get the accelerometer values
 *
 * \return the array: axes[3] of the acc_data_t structure
*/
acc_data_t* adxl345_driver_get_acc_data(void);

/**
 * \brief Initializes the accelerometer in slow mode
*/
void adxl345_driver_init_slow(void);

/**
 * \brief Returns the array: axes[3] of the acc_data_t structure in slow mode
*/
acc_data_t* adxl345_driver_get_acc_data_slow(void);

#ifdef __cplusplus
	}
#endif

#endif /* ADXL345_DRIVER_H_ */