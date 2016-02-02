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
 * \file pwm_avr32.cpp
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Julien Lecoeur
 * \author Geraud L'Eplattenier
 * \author Basil Huber
 * \author Nicolas Dousse
 * 
 * \brief This file is the driver for pwm servos
 *
 ******************************************************************************/

#include "pwm_avr32.hpp"

extern "C"
{
	#include "gpio.h"
	#include "print_util.h"
	#include "time_keeper.hpp"
	#include <math.h>
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Pwm_avr32::Pwm_avr32(uint8_t id):
	id_(id),
	channel_id_(id/2)
{
	if(id_ > 7)
	{
		id_ 		= 7;
		channel_id_ = 3; 
	}

	pulse_us_[id_]  = 1500;
	period_us_[id_] = 20000;	// 50Hz
}

bool Pwm_avr32::init(void)
{
	bool success = true;
	int32_t gpio_success;

	if( id_ > 7 || channel_id_ > 3 )
	{
		return false;
	}

	// Unlock registers
	if(channel_id_ == 0 )
	{
		AVR32_PWM.wpcr =  	( AVR32_PWM_WPCR_WPKEY_KEY << AVR32_PWM_WPCR_WPKEY ) 	|
							( AVR32_PWM_WPCR_WPRG0_MASK )							|
							( AVR32_PWM_WPCR_WPCMD_SWDIS << AVR32_PWM_WPCR_WPCMD );
	}
	else if(channel_id_ == 1 )
	{
		AVR32_PWM.wpcr = 	( AVR32_PWM_WPCR_WPKEY_KEY << AVR32_PWM_WPCR_WPKEY )	|
							( AVR32_PWM_WPCR_WPRG1_MASK )							|
							( AVR32_PWM_WPCR_WPCMD_SWDIS << AVR32_PWM_WPCR_WPCMD );
	}
	else if(channel_id_ == 2 )
	{
		AVR32_PWM.wpcr = 	( AVR32_PWM_WPCR_WPKEY_KEY << AVR32_PWM_WPCR_WPKEY )	|
							( AVR32_PWM_WPCR_WPRG2_MASK )							|
							( AVR32_PWM_WPCR_WPCMD_SWDIS << AVR32_PWM_WPCR_WPCMD );
	}
	else if(channel_id_ == 3 )
	{
		AVR32_PWM.wpcr = 	( AVR32_PWM_WPCR_WPKEY_KEY << AVR32_PWM_WPCR_WPKEY )	|
							( AVR32_PWM_WPCR_WPRG3_MASK )							|
							( AVR32_PWM_WPCR_WPCMD_SWDIS << AVR32_PWM_WPCR_WPCMD );
	}
	
    // To setup the clock  
	AVR32_PWM.clk = ( 1 <<AVR32_PWM_DIVA_OFFSET) |  // /1
				    ( 1 <<AVR32_PWM_DIVB_OFFSET) |  // /1
				    ( 6 <<AVR32_PWM_PREA_OFFSET) |  // /64
				    ( 6 <<AVR32_PWM_PREB_OFFSET) |  // /64
				    ( 0 <<AVR32_PWM_CLKSEL_OFFSET);

	// output override for low and high side
	AVR32_PWM.oov  = 	( 0b1111 << ( AVR32_PWM_OOVH0_OFFSET ) ) 	|
						( 0b1111 << ( AVR32_PWM_OOVL0_OFFSET ) );

	// output selection clear: dead time generator (0)
	AVR32_PWM.osc  = 	( 0b1111 << ( AVR32_PWM_OOVH0_OFFSET ) ) 	|
						( 0b1111 << ( AVR32_PWM_OOVL0_OFFSET ) ); 
	
	// set up channels: enable dead time insertion
	// enable dead time, set channel clock to CLKA
	AVR32_PWM.channel[channel_id_].cmr 	= AVR32_PWM_CMR0_DTE_MASK | 11;
	AVR32_PWM.channel[channel_id_].cprd = 10000;
	AVR32_PWM.channel[channel_id_].cdty = 4000;
	AVR32_PWM.channel[channel_id_].dt 	= 1000 << 16 | 1000;	

	// // Enable gpio
	if( id_ == 0 )
	{
		static const gpio_map_t PWM_GPIO_MAP =
		{
			{ AVR32_PWM_PWML_0_0_PIN, AVR32_PWM_PWML_0_0_FUNCTION },
		};			
		gpio_success = gpio_enable_module(PWM_GPIO_MAP, sizeof(PWM_GPIO_MAP) / sizeof(PWM_GPIO_MAP[0]));
	}
	else if( id_ == 1 )
	{
		static const gpio_map_t PWM_GPIO_MAP =
		{	
			{ AVR32_PWM_PWMH_0_0_PIN, AVR32_PWM_PWMH_0_0_FUNCTION },
		};			
		gpio_success = gpio_enable_module(PWM_GPIO_MAP, sizeof(PWM_GPIO_MAP) / sizeof(PWM_GPIO_MAP[0]));
	}
	else if( id_ == 2 )
	{
		static const gpio_map_t PWM_GPIO_MAP =
		{				
			{ AVR32_PWM_PWML_1_0_PIN, AVR32_PWM_PWML_1_0_FUNCTION },
		};			
		gpio_success = gpio_enable_module(PWM_GPIO_MAP, sizeof(PWM_GPIO_MAP) / sizeof(PWM_GPIO_MAP[0]));
	}
	else if( id_ == 3 )
	{
		static const gpio_map_t PWM_GPIO_MAP =
		{
			{ AVR32_PWM_PWMH_1_0_PIN, AVR32_PWM_PWMH_1_0_FUNCTION },
		};			
		gpio_success = gpio_enable_module(PWM_GPIO_MAP, sizeof(PWM_GPIO_MAP) / sizeof(PWM_GPIO_MAP[0]));
	}
	else if( id_ == 4 )
	{
		static const gpio_map_t PWM_GPIO_MAP =
		{			
			{ AVR32_PWM_PWML_2_0_PIN, AVR32_PWM_PWML_2_0_FUNCTION },
		};			
		gpio_success = gpio_enable_module(PWM_GPIO_MAP, sizeof(PWM_GPIO_MAP) / sizeof(PWM_GPIO_MAP[0]));
	}
	else if( id_ == 5 )
	{
		static const gpio_map_t PWM_GPIO_MAP =
		{
			{ AVR32_PWM_PWMH_2_0_PIN, AVR32_PWM_PWMH_2_0_FUNCTION },
		};			
		gpio_success = gpio_enable_module(PWM_GPIO_MAP, sizeof(PWM_GPIO_MAP) / sizeof(PWM_GPIO_MAP[0]));
	}
	else if( id_ == 6 )
	{
		static const gpio_map_t PWM_GPIO_MAP =
		{
			{ AVR32_PWM_PWML_3_0_PIN, AVR32_PWM_PWML_3_0_FUNCTION },
		};			
		gpio_success = gpio_enable_module(PWM_GPIO_MAP, sizeof(PWM_GPIO_MAP) / sizeof(PWM_GPIO_MAP[0]));
	}
	else if( id_ == 7 )
	{
		static const gpio_map_t PWM_GPIO_MAP =
		{
			{ AVR32_PWM_PWMH_3_0_PIN, AVR32_PWM_PWMH_3_0_FUNCTION }
		};			
		gpio_success = gpio_enable_module(PWM_GPIO_MAP, sizeof(PWM_GPIO_MAP) / sizeof(PWM_GPIO_MAP[0]));
	}

	// Enable
	AVR32_PWM.ena = 0b1111; // enable

	if (gpio_success == GPIO_SUCCESS)
	{
		success = true;
	}
	else
	{
		success = false;
	}

	return success;
}

bool Pwm_avr32::set_pulse_width_us(uint16_t pulse_us)
{
	pulse_us_[id_] = pulse_us;
	write_channel();

	return true;
}


bool Pwm_avr32::set_period_us(uint16_t period_us)
{
	period_us_[id_] = period_us;
	write_channel();

	return true;
}


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void Pwm_avr32::write_channel(void)
{
	// Set update frequency per channel with conservative method:
	// if two servos on the same channel ask for two different frequencies,
	// then the lowest frequecy is used
	int32_t period 	= max(  period_us_[2*channel_id_], 
							period_us_[2*channel_id_ + 1] ); 
	
	int32_t pulse_us_a = pulse_us_[2*channel_id_];
	int32_t pulse_us_b = pulse_us_[2*channel_id_+1];
	int32_t deadtime 	= ( period - pulse_us_a - pulse_us_b ) / 2;
	
	AVR32_PWM.channel[channel_id_ &0b11].cprdupd 	= period;
	AVR32_PWM.channel[channel_id_ &0b11].cdtyupd 	= pulse_us_a + deadtime;
	AVR32_PWM.channel[channel_id_ &0b11].dtupd 		= deadtime << 16 | deadtime;	
}

// Allocate memory for static members here
uint32_t Pwm_avr32::pulse_us_[8];
uint32_t Pwm_avr32::period_us_[8];
