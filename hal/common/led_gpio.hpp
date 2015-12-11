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
 * \file led_gpio.hpp
 * 
 * \author MAV'RIC Team
 * 
 * \brief Implementation of led using gpio 
 *
 ******************************************************************************/


#ifndef LED_GPIO_HPP_
#define LED_GPIO_HPP_

#include <stdbool.h>

#include "led.hpp"
#include "gpio.hpp"

class Led_gpio: public Led
{
public:
	/**
	 * \brief Constructor
	 * 
	 * \param gpio				Reference to gpio
	 * \param active_high		Indicates if the led is on for gpio high (true) of gpio low (false)
	 */
	Led_gpio(Gpio& gpio, bool active_high = true);


	/**
	 * \brief	Switch led on
	 */
	void on(void);


	/**
	 * \brief	Switch led off
	 */
	void off(void);


	/**
	 * \brief	Toggle led
	 */
	void toggle(void);


private:
	Gpio&	gpio_;			//< Reference to gpio

	bool 	active_high_;	///< Indicates if the led is on for gpio high (true) of gpio low (false)
};


#endif /* LED_GPIO_HPP_ */
