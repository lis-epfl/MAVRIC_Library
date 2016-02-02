/*******************************************************************************
 * Copyright (c) 2009-2016, MAV'RIC Development Team
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
 * \file gps.hpp
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief Abstract class for GPS
 * 
 ******************************************************************************/


#ifndef GPS_HPP_
#define GPS_HPP_


#include <array>

extern "C"
{
	#include "util/coord_conventions.h"
}


/**
 * \brief Abstract class for GPS
 */
class Gps
{
public:
	/**
	 * \brief 	Main update function
	 * \detail 	Reads new values from sensor
	 * 
	 * \return 	Success
	 */
	virtual bool update(void) = 0;


	/**
	 * \brief 	Configure the GPS
	 */
	virtual void configure(void) = 0;

	
	/**
	 * \brief 	Get last update time in microseconds
	 * 
	 * \return 	Update time
	 */
	virtual const float& last_update_us(void) const = 0;


	/**
	 * \brief 	Get last position update time in microseconds
	 * 
	 * \return 	Update time
	 */
	virtual const float& last_position_update_us(void) const = 0;


	/**
	 * \brief 	Get last velocity update time in microseconds
	 * 
	 * \return 	Update time
	 */
	virtual const float& last_velocity_update_us(void) const = 0;


	/**
	 * \brief 	Get position in global frame
	 * 
	 * \return 	position
	 */	
	virtual const global_position_t& position_gf(void) const = 0;


	/**
	 * \brief 	Get horizontal position accuracy in m
	 * 
	 * \return 	accuracy
	 */	
	virtual const float& horizontal_position_accuracy(void) const = 0;


	/**
	 * \brief 	Get vertical position accuracy in m
	 * 
	 * \return 	accuracy
	 */	
	virtual const float& vertical_position_accuracy(void) const = 0;


	/**
	 * \brief 	Get velocity in local frame in m/s
	 * 
	 * \return 	3D velocity
	 */	
	virtual const std::array<float,3>& velocity_lf(void) const = 0;


	/**
	 * \brief 	Get velocity accuracy in m/s
	 * 
	 * \return 	velocity accuracy
	 */	
	virtual const float& velocity_accuracy(void) const = 0;


	/**
	 * \brief 	Get heading in degrees
	 * 
	 * \return 	heading
	 */	
	virtual const float& heading(void) const = 0;


	/**
	 * \brief 	Get heading accuracy in degrees
	 * 
	 * \return 	accuracy
	 */	
	virtual const float& heading_accuracy(void) const = 0;


	/**
	 * \brief 	Get the number of satellites
	 * 
	 * \return 	Value
	 */
	virtual const uint8_t& num_sats(void) const = 0;


	/**
	 * \brief 	Indicates whether fix are received
	 * 
	 * \return 	Value
	 */
	virtual const bool& fix(void) const = 0;


	/**
	 * \brief 	Indicates whether the measurements can be trusted 
	 * 
	 * \return 	Value
	 */
	virtual const bool& healthy(void) const = 0;
};


#endif /* GPS_HPP_ */