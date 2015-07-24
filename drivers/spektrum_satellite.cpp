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
 * \file spektrum_satellite.c
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Julien Lecoeur
 *   
 * \brief This file is the driver for the remote control
 * 
 ******************************************************************************/


#include "spektrum_satellite.hpp"

extern "C" 
{
	#include "spektrum.h"

	#include "usart.h"

	#include "time_keeper.h"
	#include "gpio.h"
	#include "sysclk.h"

	#include "print_util.h"

	// #include "delay.h"

	#include "led.h"
}

const uint8_t DSM_RECEIVER_PIN 			= AVR32_PIN_PD12;		///< Define the microcontroller pin map with the receiver pin
const uint8_t RECEIVER_POWER_ENABLE_PIN = AVR32_PIN_PC01;		///< Define the microcontroller pin map with the receiver power enable pin


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief Power-on the receiver
 */
void spektrum_satellite_switch_on(void);


/**
 * \brief Power-off the receiver
 */
void spektrum_satellite_switch_off(void);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void spektrum_satellite_switch_on(void) 
{
	gpio_configure_pin(RECEIVER_POWER_ENABLE_PIN, GPIO_DIR_OUTPUT);
	gpio_set_pin_low(RECEIVER_POWER_ENABLE_PIN);
}


void spektrum_satellite_switch_off(void) 
{
	gpio_configure_pin(RECEIVER_POWER_ENABLE_PIN, GPIO_DIR_OUTPUT);
	gpio_set_pin_high(RECEIVER_POWER_ENABLE_PIN);
}


/**
 * \brief Define the service routine for the spektrum handler interruption
 */
 /*
#pragma interrupt
ISR(spectrum_handler, AVR32_USART1_IRQ, AVR32_INTC_INTLEV_INT1) 
{
	uint8_t c1, c2, i;
	uint16_t sw;
	uint8_t channel;
	uint32_t now = time_keeper_get_micros() ;

	// If byte received
	if (usart_conf_sat_.uart_device.uart->csr & AVR32_USART_CSR_RXRDY_MASK) 
	{
		uint32_t dt_interrupt = now - last_interrupt_;
		last_interrupt_ = now;

		// the shorter frame period is 11'000us (11bits encoding) and the longer frame period is 22'000 us(10bits encoding) 
		// the inter byte period within a frame is 77us
		// Clear buffer if the new byte of the on-going frame came after 2times the inter-byte period
		if ( (buffer_bytes_available(&receiver_)!=0) && (dt_interrupt > 150))
		{
			buffer_clear(&receiver_);
		}

		// Add new byte to buffer
		c1 = (uint8_t)usart_conf_sat_.uart_device.uart->rhr;
		buffer_put(&receiver_, c1);
		
		// If frame is complete, decode channels
		if ( (buffer_bytes_available(&receiver_) == 16) && (protocol_ != UNKNOWN)) 
		{
			// first two bytes are status info,
			c1 = buffer_get(&receiver_);
			c2 = buffer_get(&receiver_);
			
			if ((protocol_ == DSM2_10BITS) && ((c1 != 0x03) || (c2 != 0xB2))) //correspond to DSM2 10bits header
			{
				buffer_clear(&receiver_);
				return;
			}
				
			for (i = 0; i < 7; i++) // 7 channels per frame
			{
				c1 = buffer_get(&receiver_);
				c2 = buffer_get(&receiver_);
				sw = (uint16_t)c1 << 8 | ((uint16_t)c2);
								
				if ( protocol_ == DSM2_10BITS )  //10 bits
				{
					// highest bit is frame 0/1, bits 2-6 are channel number
					channel = ((sw >> 10))&0x0f;
					
					// 10 bits per channel
					channels_[channel] = ((int16_t)(sw&0x3ff) - 512) * 2;
				} 
				else if ( protocol_ == DSM2_11BITS ) //11bits
				{
					// highest bit is frame 0/1, bits 3-7 are channel number
					channel = ((sw >> 11))&0x0f;
					
					// 11 bits per channel
					channels_[channel] = ((int16_t)(sw&0x7ff) - 1024);
				}
			}//end of for loop	
		
			// update timing
			dt_ 			= now - last_update_;
			last_update_	= now;

			// Inidicate that new data is available
			new_data_available_ = true;
		}
		else if ( buffer_bytes_available(&receiver_) == 16)
		{
			//Since protocol is unknown
			//check the radio protocol
			
			// first two bytes are status info,
			c1 = buffer_get(&receiver_);
			c2 = buffer_get(&receiver_);
			
			if (c1 == 0x03 && c2 == 0xB2) //correspond to DSM2 10bits header
			{
				protocol_proba_.proba_10bits++;
				//empty_the buffer, since we don't decode channels yet
				buffer_clear(&receiver_);
			}
			else
			{
				protocol_proba_.proba_11bits++;
				//empty_the buffer, since we don't decode channels yet
				buffer_clear(&receiver_);
			}
			
			//after having received enough frames, determine which protocol is used
			if (protocol_proba_.min_nb_frames != 0 )
			{
				protocol_proba_.min_nb_frames--;
			}
			else
			{
				//is the probability of one protocol at least 2 times bigger than for the other one ?
				if (protocol_proba_.proba_10bits > 2*protocol_proba_.proba_11bits)
				{
					protocol_ = DSM2_10BITS;
				}
				else if (protocol_proba_.proba_11bits > 2*protocol_proba_.proba_10bits)
				{
					protocol_ = DSM2_11BITS;
				}
				else //otherwise redo this probability check for 10 other frames
				{
					protocol_proba_.min_nb_frames = 10;
				}
				
			}
			
		}
	}		
}
*/

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Spektrum_satellite::Spektrum_satellite(Serial_avr32& uart):
	uart_(uart)
{}

bool Spektrum_satellite::init() 
{	
	bool result = true; //cannot go wrong...

	for (int32_t i = 0; i < 16; i++) 
	{
		channels_[i] = 0;
		channel_center_[i] = 0;
	}
	
	new_data_available_ = false;
	protocol_ = UNKNOWN;
	last_update_ = time_keeper_get_micros();
	//Set minimum number of frames to be received in order to guess the radio protocol used
	protocol_proba_.min_nb_frames = 10; 
    protocol_proba_.proba_10bits = 0;
    protocol_proba_.proba_11bits = 0;
    
	spektrum_satellite_switch_on();

	return result;
}

void Spektrum_satellite::bind(radio_protocol_t protocol)
{
	int32_t i = 0;
	// uint32_t cpu_freq = sysclk_get_cpu_hz();

	print_util_dbg_print(" \n receive bind CMD \n");
	
	// Switch off satellite
	spektrum_satellite_switch_off();
	time_keeper_delay_ms(100);
	
	//set as input, pull down not to be floating
	gpio_configure_pin(DSM_RECEIVER_PIN, GPIO_DIR_INPUT | GPIO_PULL_DOWN);	

	spektrum_satellite_switch_on();

	// Wait for startup signal
	while ((gpio_get_pin_value(DSM_RECEIVER_PIN) == 0) && (i < 10000)) 
	{
		i++;
		time_keeper_delay_ms(1);
	}
	
	// Wait 100ms after receiver startup
	time_keeper_delay_ms(68);
	
	uint8_t pulses = 0;
	if (protocol == DSM2_10BITS)
	{
		pulses = 3;
	}
	else if (protocol == DSM2_11BITS)
	{
		pulses = 6;
	}

	// create 6 pulses with 250us period to set receiver to bind mode
	for (i = 0; i < pulses; i++) 
	{
		gpio_configure_pin(DSM_RECEIVER_PIN, GPIO_DIR_OUTPUT | GPIO_PULL_DOWN);
		// cpu_delay_us(113, cpu_freq); 
		time_keeper_delay_micros(113);
		gpio_configure_pin(DSM_RECEIVER_PIN, GPIO_DIR_INPUT | GPIO_PULL_UP);	
		// cpu_delay_us(118, cpu_freq);
		time_keeper_delay_micros(118);
	}
}


int16_t Spektrum_satellite::get_channel(const uint8_t index) const
{
	return channels_[index];
}


int16_t Spektrum_satellite::get_neutral(const uint8_t index) const
{
	int16_t value = get_channel(index) - channel_center_[index];

 	// clamp to dead zone
 	if ( (value > -DEADZONE) && (value < DEADZONE) )
 	{
 		value=0;
 	}

 	return value;
 }
