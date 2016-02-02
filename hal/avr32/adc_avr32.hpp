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
 * \file 	adc_avr32.hpp
 * 
 * \author 	MAV'RIC Team
 *   
 * \brief 	Implementation of Analog to digital converters on avr32
 *
 ******************************************************************************/

#ifndef ADC_AVR32_HPP_
#define ADC_AVR32_HPP_

#include "hal/common/adc.hpp"

extern "C"
{
	#include "hal/analog_monitor.h"
}


/**
 * \brief 	Configuration for Adc_avr32
 */
typedef struct
{
	analog_rails_t rail;	///< Analog rail to use
} adc_avr32_conf_t;


/**
 * \brief 	Implementation of Analog to digital converters on avr32 * 
 */
class Adc_avr32: public Adc
{
public:
	/**
	 * \brief  	Constructor
	 * 
	 * \param 	analog_monitor	Reference to analog monitor
	 * \param 	config 			Device configuration
	 */
	Adc_avr32(analog_monitor_t& analog_monitor, adc_avr32_conf_t config);
	
	/**
	 * \brief 	Hardware initialization
	 * 
	 * \return  success
	 */
	bool init(void);

	/**
	 * \brief 	Update function
	 * 
	 * \return  success
	 */
	bool update(void);


	/**
	 * \brief 	Get the voltage in volts
	 * 
	 * \return	Value
	 */	
	const float& voltage(void) const;

private:
	analog_monitor_t& 	analog_monitor_;		///< Reference to analog monitor structure
	adc_avr32_conf_t 	config_;				///< Configuration

	float 				voltage_;				///< Current voltage
};


#endif /* ADC_AVR32_HPP_ */