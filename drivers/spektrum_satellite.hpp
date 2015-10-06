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


#ifndef SPEKTRUM_SATELLITE_HPP_
#define SPEKTRUM_SATELLITE_HPP_


#include "satellite.hpp"
#include "serial.hpp"
 
extern "C" 
{
	#include <stdint.h>
	#include <stdbool.h>
	#include "buffer.h"
}


/**
 * \brief Structure containing the radio protocol probabilities
 */
typedef struct
{
	uint8_t min_nb_frames;		///< Minimum of Frames used to determine the protocol used
	uint8_t proba_10bits;	///< Probability that the protocol is 10bits
	uint8_t	proba_11bits;	///< Probability that the protocol is 11bits
} dsm2_protocol_proba_t;


class Spektrum_satellite : public Satellite
{
public:
	/**
	* @brief  	Constructor
	* 
	* @param 	uart 	Reference to UART device 
	*/
	Spektrum_satellite(Serial& uart);


	/**
	* \brief Initialize UART receiver for Spektrum/DSM2 slave receivers
	*/
	bool init ();


	/**
	* \brief Sets the satellite in bind mode
	*
	* \param	protocol	Channel encoding. It it either 10bits (up to 7 channels) or 11bits(up to 14 channels). We support both. 
	*
	*/
	void bind(radio_protocol_t protocol);


	/**
	* \brief 	Return a channels' value
	*
	* \param 	channel_number		The channel ID
	*
	* \return 	Value for channel channel_number
	*/
	int16_t	channel(const uint8_t channel_number) const;


	/**
	* \brief 	Return the last update time in microseconds
	*
	* \return 	Last update time
	*/
	uint32_t last_update(void) const;

	
	/**
	* \brief 	Return the time difference between the last 2 updates in microseconds
	*
	* \return 	dt
	*/
	uint32_t dt(void) const;


	/**
	 * \brief  			Takes care of incoming data
	 */
	void handle_interrupt();


private:
	Serial&					uart_;					///< Serial port
	buffer_t 				receiver_;				///< Buffer for incoming data
	int16_t 				channels_[16];			///< Array to contain the 16 remote channels
	uint32_t 				last_interrupt_;		///< Last time a byte was received
	uint32_t 				last_update_;			///< Last update time 
	uint32_t 				dt_;					///< Duration between two updates
	bool					new_data_available_; 	///< Indicates if new data is  available
	dsm2_protocol_proba_t	protocol_proba_;		///< Indicates number of frames received
	radio_protocol_t		protocol_;				///< Defines in which mode the remote is configured

};



#endif /* SPEKTRUM_SATELLITE_HPP_ */
