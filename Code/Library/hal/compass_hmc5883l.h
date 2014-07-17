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
* \file compass_hmc58831l.h
*
* This file is the driver for the magnetometer HMC58831
*/


#ifndef COMPASS_HMC5883L_H_
#define COMPASS_HMC5883L_H_

#ifdef __cplusplus
	extern "C" {
#endif

#include <stdint.h>
#include "compass.h"


/**
 * \brief structure for the magnetometer's data
*/
typedef struct
{
	compass_data_t *raw_compass;
} compass_hmc58831l_data_t;


/**
 * \brief Initializes the magnetometer sensor
*/
void compass_hmc58831l_init(void);


/**
 * \brief Initializes the magnetometer sensor in slow mode
*/
void compass_hmc58831l_init_slow(void);


/**
 * \brief Returns the magnetometer's data in slow mode
 *
 * \return a pointer to the magnetometer's data in slow mode
*/
void compass_hmc58831l_update(compass_data_t *compass_outputs);


#ifdef __cplusplus
	}
#endif

#endif /* COMPASS_HMC5883_H_ */