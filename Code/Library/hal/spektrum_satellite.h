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
	buffer_t 		receiver;			///< Buffer for incoming data
	int16_t 		channels[16];		///< Array to contain the 16 remote channels
	uint32_t 		last_interrupt;		///< Last time a byte was received
	uint32_t 		last_update;		///< Last update time 
	uint32_t 		dt;					///< Duration between two updates
	bool			new_data_available; ///< Indicates if new data is  available
} spektrum_satellite_t;


/**
 * \brief set slave receiver into bind mode. 
 * has to be called 100ms after power-up
 */
void spektrum_satellite_bind(void);


/**
 * \brief Initialize UART receiver for Spektrum/DSM2 slave receivers
 */
void spektrum_satellite_init(void);


spektrum_satellite_t* spektrum_satellite_get_pointer(void);


/**
 * \brief Return a remote channel
 *
 * \param index Specify which channel we are interested in
 *
 * \return the remote channel value
 */
int16_t spektrum_satellite_get_channel(uint8_t index);


/**
 * \brief 	Return the a remote channel taking neutral into account
 *
 * \param 	index 		Specify which channel we are interested in
 *
 * \return 				
 */
int16_t spektrum_satellite_get_neutral(uint8_t index);


#ifdef __cplusplus
	}
#endif

#endif /* REMOTE_DSM2_ */
