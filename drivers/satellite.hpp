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
 * \file satellite.hpp
 * 
 * \author MAV'RIC Team
 * \author Gregoire Heitz
 *   
 * \brief This declare the global satellite struct
 * enable usage of different satellite receiver (ie. spektrum, emulated...)
 *
 ******************************************************************************/

#ifndef SATELLITE_HPP_
#define SATELLITE_HPP_


extern "C" 
{
	#include "uart_int.h"
}

/**
 * \brief Structure containing the radio protocol probabilities
 */
typedef struct
{
	uint8_t min_nb_frames;	///< Minimum of Frames used to determine the protocol used
	uint8_t proba_10bits;	///< Probability that the protocol is 10bits
	uint8_t	proba_11bits;	///< Probability that the protocol is 11bits
}radio_protocol_proba_t;


/**
 * \brief Radio protocols
 */ 
typedef enum
{
	DSM2_10BITS = 0,
	DSM2_11BITS = 1,
	DSMX		= 2,
	UNKNOWN		= 3,
} radio_protocol_t;

//Function pointer

class Satellite
{
public:
	Satellite();

	int16_t 	get_channels(const uint8_t channel_number) const;

	uint32_t 	get_last_interrupt(void) const;

	uint32_t 	get_last_update(void) const;

	uint32_t 	get_dt(void) const;

	bool 		get_new_data_available(void) const;
	void 		set_new_data_available(const bool is_available);

	usart_config_t	get_usart_conf_sat(void) const;

	virtual void	bind(const radio_protocol_t radio_protocol);

	virtual bool	init();

protected:
	buffer_t 				receiver_;				///< Buffer for incoming data
	int16_t 				channels_[16];			///< Array to contain the 16 remote channels
	uint32_t 				last_interrupt_;		///< Last time a byte was received
	uint32_t 				last_update_;			///< Last update time 
	uint32_t 				dt_;					///< Duration between two updates
	bool					new_data_available_; 	///< Indicates if new data is  available
	radio_protocol_proba_t	protocol_proba_;		///< Indicates number of frames received
	radio_protocol_t		protocol_;				///< Defines in which mode the remote is configured
	usart_config_t			usart_conf_sat_;		///< store UART conf for satellite com
};


#endif //SATELLITE_HPP_
