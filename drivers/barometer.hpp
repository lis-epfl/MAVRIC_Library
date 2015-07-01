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
 * \file barometer.h
 * 
 * \author MAV'RIC Team
 * \author Gregoire Heitz
 *   
 * \brief This file defines the barometer enum and structure, independently from which sensor is used
 *
 ******************************************************************************/


#ifndef BAROMETER_HPP_
#define BAROMETER_HPP_

/**
 * \brief bmp085_state_t can get three different state: Idle, get Temperature or get Pressure
*/
typedef enum bmp085_state_t
{
	IDLE,								///< Idle state
	GET_TEMP,							///< Getting temperature state
	GET_PRESSURE						///< Getting pressure state
} barometer_state_t;


class Barometer
{
public:
	/**
	 * @brief  	Constructor
	 */
	Barometer();

	/**
	 * @brief   Reset the altitude to position estimation origin
	 * 
	 * @param	origin_altitude 	Altitude corresponding to the origin
	 */
	void reset_origin_altitude(float origin_altitude);
	
	/**
	 * @brief   Return the pressure
	 * 
	 * @return 	the pressure member
	 */
	float get_pressure(void)  const;

	 /**
	 * @brief   Return the altitude speed
	 * 
	 * @return 	the vario_vz_ member
	 */
	 float get_vario_vz(void) const;

	 /**
	 * @brief   Return the temperature
	 * 
	 * @return 	the temperature member
	 */
	 float get_temperature(void) const;

	 /**
	 * @brief   Return the last update time
	 * 
	 * @return 	the temperature member
	 */
	 uint32_t get_last_update(void) const;

	 /**
	 * @brief   Return the altitude
	 * 
	 * @return 	the temperature member
	 */
	 float get_altitude(void) const;

	 /**
	 * @brief   Return the altitude
	 * 
	 * @return 	the temperature member
	 */
	 float get_altitude_offset(void) const;

	 // FOR SIMULATION PURPOSES ONLY
	 /**
	 * @brief   Set the simulated altitude speed
	 * 
	 * @param	origin_altitude 	Altitude corresponding to the origin
	 *
	 * @return 	the vario_vz_ member
	 */
	 void set_vario_vz(float simulated_vario_vz);

	 /**
	 * @brief   Set the simulated altitude
	 * 
	 * @param	origin_altitude 	Altitude corresponding to the origin
	 *
	 * @return 	the temperature member
	 */
	 void set_altitude(float simulated_altitude);

	 /**
	 * @brief   Set the simulated altitude offset
	 * 
	 * @param	origin_altitude 	Altitude corresponding to the origin
	 *
	 * @return 	the temperature member
	 */
	 void set_altitude_offset(float simulated_altitude_offset);

	 /**
	 * @brief   Return the last update time
	 * 
	 * @param	origin_altitude 	Altitude corresponding to the origin
	 *
	 * @return 	the temperature member
	 */
	 void set_last_update(uint32_t simulated_last_update);

protected:
	uint8_t 			raw_pressure_[3];		///< Raw pressure contained in 3 uint8_t
	uint8_t 			raw_temperature_[2];	///< Raw temperature contained in 2 uint8_t
	
	float 				pressure_;				///< Measured pressure as the concatenation of the 3 uint8_t raw_pressure
	float 				temperature_;			///< Measured temperature as the concatenation of the 2 uint8_t raw_temperature
	float 				altitude_;				///< Measured altitude as the median filter of the 3 last_altitudes
	float 				altitude_offset_;		///< Offset of the barometer sensor for matching GPS altitude value
	float 				vario_vz_;				///< Vario altitude speed
	
	float 				last_altitudes_[3];		///< Array to store previous value of the altitude for low pass filtering the output
	
	uint32_t 			last_update_;			///< Time of the last update of the barometer
	uint32_t 			last_state_update_;		///< Time of the last state update
	barometer_state_t 	state_;			///< State of the barometer sensor (IDLE, GET_TEMP, GET_PRESSURE)
	float 				dt_;					///< Time step for the derivative
};

#endif /* BAROMETER_HPP_ */
