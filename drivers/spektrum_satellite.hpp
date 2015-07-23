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


#ifndef SPEKTRUM_SATELLITE_
#define SPEKTRUM_SATELLITE_


#include "satellite.hpp"
#include "serial_avr32.hpp"

extern "C" 
{
	#include <stdint.h>
	#include <stdbool.h>
	#include "buffer.h"
}


class Spektrum_satellite : public Satellite
{
public:
	/**
	* @brief  	Constructor
	* 
	* @param 	uart 	Reference to UART device 
	*/
	Spektrum_satellite(Serial_avr32& uart);

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
	* \brief Return a remote channel
	*
	* \param	index	Specify which channel we are interested in
	*
	* \return the remote channel value
	*/
	int16_t get_channel(const uint8_t index) const;


	/**
	* \brief 	Return the a remote channel taking neutral into account
	*
	* \param 	index 	Specify which channel we are interested in
	*
	* \return 	 the remote channel neutral value		
	*/
	int16_t get_neutral(const uint8_t index) const;

private:
	void interrupt(void);

	Serial_avr32& 	uart_;
	int16_t 		channel_center_[16];		///< Declare an array to store the central position of each channel

};



#endif /* SPEKTRUM_SATELLITE_ */
