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
* \file spektrum_satellite.h
*
* This file is the driver for the remote control
*/


#ifndef REMOTE_DSM2_
#define REMOTE_DSM2_

#ifdef __cplusplus
	extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "buffer.h"


/**
 * \brief Structure containing the Spektrum receiver's data
 */
typedef struct 
{
	buffer_t receiver;							///< Define a buffer for the receiver
	int16_t channels[16];						///< Define an array to contain the 16 remote channels availbale
	uint32_t last_update;						///< Define the last update time 
	uint8_t valid;								///< Define whether a data is valid or not
	uint32_t last_time;							///< Define last time
	uint32_t duration;							///< Define the duration
} spektrum_satellite_t;


/**
 * \brief Power-on the receiver
 */
void spektrum_satellite_switch_on(void);


/**
 * \brief Power-off the receiver
 */
void spektrum_satellite_switch_off(void);


/**
 * \brief set slave receiver into bind mode. 
 * has to be called 100ms after power-up
 */
void spektrum_satellite_bind(void);


/**
 * \brief Initialize UART receiver for Spektrum/DSM2 slave receivers
 */
void spektrum_satellite_init(void);


/**
 * \brief Return a remote channel
 *
 * \param index Specify which channel we are interested in
 *
 * \return the remote channel value
 */
int16_t spektrum_satellite_get_channel(uint8_t index);


/**
 * \brief Update the remote channel central position array stored in .c file
 * Warning: you should ensure first that the remote has the stick in their neutral position first
 *
 * \param index Specify which channel we are interested in
 */
void spektrum_satellite_calibrate_center(uint8_t index);


/**
 * \brief 	Return the a remote channel taking neutral into account
 *
 * \param 	index 		Specify which channel we are interested in
 *
 * \return 				
 */
int16_t spektrum_satellite_get_neutral(uint8_t index);


/**
 * \brief return 1 if enabled receivers works
 *
 * \return the error value of receivers' checks: 0 for no error
 */
int8_t  spektrum_satellite_check(void);


#ifdef __cplusplus
	}
#endif

#endif /* REMOTE_DSM2_ */
