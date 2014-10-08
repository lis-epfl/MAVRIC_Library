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
 * \file spektrum_satellite.h
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Julien Lecoeur
 *   
 * \brief This file is the driver for the remote control
 * 
 ******************************************************************************/


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
 * \brief Initialize UART receiver for Spektrum/DSM2 slave receivers
 */
void spektrum_satellite_init(void);

/**
 * \brief Sets the satellite in bind mode
 */
void spektrum_satellite_bind(void);

/**
 * \brief	Return a pointer to the satellite structure
 *
 * \return The pointer to the satellite structure
 */
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
