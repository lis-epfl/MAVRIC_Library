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
* \file itg3200_driver.h
*
* This file is the driver for the integrated triple axis gyroscope ITG3200
*/

#ifndef ITG3200_DRIVER_H_
#define ITG3200_DRIVER_H_

#ifdef __cplusplus
	extern "C" {
#endif

#include "compiler.h"

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
} gyro_data; 


/**
 * \brief Initialize the gyroscope sensor
 * not implemented yet
*/
void init_itg3200(void);

/**
 * \brief Reconfigure the gyroscope sensor
*/
void reconfigure_gyro(void);

/**
 * \brief Return the gyroscope's data in normal mode
 * not implemented yet
 *
 * \return a pointer to the gyroscope data
*/
gyro_data* get_gyro_data(void);

/**
 * \brief Initialize the gyroscope sensor in slow mode
*/
void init_itg3200_slow(void);

/**
 * \brief Return the gyroscope's data in slow mode
 *
 * \return a pointer to the gyroscope data
*/
gyro_data* get_gyro_data_slow(void);

#ifdef __cplusplus
	}
#endif

#endif /* ITG3200_DRIVER_H_ */