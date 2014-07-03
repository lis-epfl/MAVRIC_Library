/**
 * \page The MAV'RIC License
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */


/**
* \file adxl345_driver.h
*
* This file is the Driver for the ADXL345 accelerometer
*/


#ifndef ADXL345_DRIVER_H_
#define ADXL345_DRIVER_H_

#ifdef __cplusplus
	extern "C" {
#endif

#include "compiler.h"

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
} acc_data; 

/**
 * \brief Initializes the accelerometer
*/
void init_adxl345(void);

/**
 * \brief Get the accelerometer values
 *
 * \return the array: axes[3] of the acc_data structure
*/
acc_data* get_acc_data(void);

/**
 * \brief Initializes the accelerometer in slow mode
*/
void init_adxl345_slow(void);

/**
 * \brief Returns the array: axes[3] of the acc_data structure in slow mode
*/
acc_data* get_acc_data_slow(void);

#ifdef __cplusplus
	}
#endif

#endif /* ADXL345_DRIVER_H_ */