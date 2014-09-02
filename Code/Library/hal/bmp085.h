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
* \file bmp085.h
*
* This file is the driver for the barometer module: BMP085
*/


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
	barometer_t* barometer;
} bmp085_t;


/**
 * \brief Initialize the barometer sensor
*/
void bmp085_init(barometer_t *bmp085, const mavlink_stream_t * mavlink_stream);


/**
 * \brief Initialize the barometer sensor in slow mode
*/
void bmp085_init_slow(void);


/**
 * \brief	Initialization of the pos_est->barometer offset
 *
 * \param	pos_est			The pointer to the position estimation structure
 * \param	pos_est->barometer		The pointer to the pos_est->barometer structure
 *
 * \return	void
 */
void bmp085_reset_origin_altitude(barometer_t* bmp085, float origin_altitude);


/**
 * \brief Get the pressure data n slow mode
 *
 * \param offset Offset to add to the pressure measured by the barometer
 *
 * \return a pointer to the pressure data structure
*/
void bmp085_update(barometer_t *bmp085);


/**
 * \brief	Task to send the mavlink scaled pressure message
 * 
 * \param	pressure	The pointer to the pressure structure
 *
 * \return	The status of execution of the task
 */
task_return_t bmp085_send_pressure(barometer_t* bmp085);


#ifdef __cplusplus
}
#endif

#endif /* BMP085_H_ */