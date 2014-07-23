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
* \file hmc5883l.h
*
* This file is the driver for the magnetometer HMC58831
*/


#ifndef HMC5883L_H_
#define HMC5883L_H_

#ifdef __cplusplus
	extern "C" {
#endif

#include <stdint.h>
#include "magnetometer.h"


/**
 * \brief structure for the magnetometer's data
*/
typedef struct
{
	magnetometer_t *raw_compass;
} hmc5883l_t;


/**
 * \brief Initializes the magnetometer sensor
*/
void hmc5883l_init(void);


/**
 * \brief Initializes the magnetometer sensor in slow mode
*/
void hmc5883l_init_slow(void);


/**
 * \brief Returns the magnetometer's data in slow mode
 *
 * \return a pointer to the magnetometer's data in slow mode
*/
void hmc5883l_update(magnetometer_t *compass_outputs);


#ifdef __cplusplus
	}
#endif

#endif /* COMPASS_HMC5883_H_ */