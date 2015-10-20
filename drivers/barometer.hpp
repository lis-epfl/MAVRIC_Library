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
 * \file barometer.hpp
 * 
 * \author MAV'RIC Team
 * \author Gregoire Heitz
 * \author Julien Lecoeur
 *   
 * \brief Abstract class for barometers
 *
 ******************************************************************************/


#ifndef BAROMETER_HPP_
#define BAROMETER_HPP_


/**
 * \brief 	Abstract class for barometers
 */
class Barometer
{
public:
	/**
	 * \brief   Initialise the sensor
	 * 			
	 * \return 	Success
	 */	
	virtual bool init(void) = 0;


	/**
	 * \brief 	Main update function
	 * \detail 	Reads new values from sensor
	 * 
	 * \return 	Success
	 */
	virtual bool update(void) = 0;


	 /**
	 * \brief   Get the last update time in microseconds
	 * 
	 * \return 	Value
	 */
	virtual const float& last_update_us(void) const = 0;


	/**
	 * \brief   Return the pressure
	 * 
	 * \return 	Value
	 */
	virtual const float& pressure(void)  const = 0;


	/**
	 * \brief   Get the altitude in meters
	 * 
	 * \detail 	Not NED frame: (>0 means upward)
	 * 
	 * \return 	Value
	 */
	virtual const float& altitude(void) const = 0;


	/**
	 * \brief   Get the vertical speed in meters/second
	 * 
	 * \detail 	Not NED frame: (>0 means upward)
	 * 
	 * \return 	Value
	 */
	virtual const float& vario_vz(void) const = 0;


	/**
	 * \brief   Get sensor temperature
	 * 
	 * \return 	Value
	 */
	virtual const float& temperature(void) const = 0;


	/**
	 * \brief   Reset the origin altitude
	 * 
	 * \param	origin_altitude 	New origin altitude
	 * 
	 * \return 	success
	 */
	virtual bool reset_origin_altitude(float origin_altitude) = 0;
};

#endif /* BAROMETER_HPP_ */



	//  /**
	//  * \brief   Return the altitude
	//  * 
	//  * \return 	the temperature member
	//  */
	// virtual  float altitude_offset(void) const = 0;


	 // FOR SIMULATION PURPOSES ONLY
	//  /**
	//  * \brief   Set the simulated altitude speed
	//  * 
	//  * \param	origin_altitude 	Altitude corresponding to the origin
	//  *
	//  * \return 	the vario_vz_ member
	//  */
	// virtual  void set_vario_vz(float simulated_vario_vz) = 0;


	 // /**
	 // * \brief   Set the simulated altitude
	 // * 
	 // * \param	origin_altitude 	Altitude corresponding to the origin
	 // *
	 // * \return 	the temperature member
	 // */
	 // void set_altitude(float simulated_altitude);

	 // /**
	 // * \brief   Set the simulated altitude offset
	 // * 
	 // * \param	origin_altitude 	Altitude corresponding to the origin
	 // *
	 // * \return 	the temperature member
	 // */
	 // void set_altitude_offset(float simulated_altitude_offset);

	 // /**
	 // * \brief   Return the last update time
	 // * 
	 // * \param	origin_altitude 	Altitude corresponding to the origin
	 // *
	 // * \return 	the temperature member
	 // */
	 // void set_last_update(uint32_t simulated_last_update);