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
 * \file 	gpio_avr32.cpp
 * 
 * \author 	MAV'RIC Team
 *   
 * \brief 	Implementation of GPIO peripherals for avr32
 *
 ******************************************************************************/


#include "gpio_avr32.hpp"
#include "gpio.h"

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Gpio_avr32::Gpio_avr32(gpio_avr32_conf_t config)
{
	config_ = config;
}


bool Gpio_avr32::init(void)
{
	bool success = true;

	success &= configure(config_.dir, config_.pull);

	return success;
}


bool Gpio_avr32::configure(gpio_dir_t dir, gpio_pull_updown_t pull)
{
	uint32_t flags = 0;

	// Keep config
	config_.dir  = dir;
	config_.pull = pull;

	// Get Atmel flags
	switch( dir )
	{
		case GPIO_INPUT:
			flags |= GPIO_DIR_INPUT;
		break;

		case GPIO_OUTPUT:
			flags |= GPIO_DIR_OUTPUT;
		break;
	}

	switch( pull )
	{
		case GPIO_PULL_UPDOWN_NONE:
		break;

		case GPIO_PULL_UPDOWN_UP:
			flags |= GPIO_PULL_UP;
		break;

		case GPIO_PULL_UPDOWN_DOWN:
			flags |= GPIO_PULL_DOWN;
		break;
	}

	// Write to pin
	gpio_configure_pin(config_.pin, flags);

	return true;	
}

bool Gpio_avr32::set_high(void)
{
	gpio_set_pin_high(config_.pin);

	return true;
}


bool Gpio_avr32::set_low(void)
{
	gpio_set_pin_low(config_.pin);

	return true;
}


bool Gpio_avr32::toggle(void)
{
	gpio_toggle_pin(config_.pin);

	return true;
}


bool Gpio_avr32::write(bool level)
{
	if( level == false )
	{
		set_low();
	}
	else
	{
		set_high();
	}

	return true;
}


bool Gpio_avr32::read(void)
{
	return gpio_get_pin_value(config_.pin);
}


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------