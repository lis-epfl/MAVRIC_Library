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


#include "spektrum_satellite.h"
#include "spektrum.h"

#include "usart.h"

#include "time_keeper.h"
#include "gpio.h"
#include "sysclk.h"

#include "print_util.h"

#include "delay.h"

#include "led.h"


#define REMOTE_UART AVR32_USART1						///< Define the microcontroller pin map with the remote UART
#define DSM_RECEIVER_PIN AVR32_PIN_PD12					///< Define the microcontroller pin map with the receiver pin
#define RECEIVER_POWER_ENABLE_PIN AVR32_PIN_PC01		///< Define the microcontroller pin map with the receiver power enable pin
#define BAUD_REMOTE  115200								///< Define the remote baudrate

spektrum_satellite_t sat;								///< Declare an object containing the receiver structure for receiver 1

int16_t channel_center[16];								///< Declare an array to store the central position of each channel


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
ISR(spectrum_handler, AVR32_USART1_IRQ, AVR32_INTC_INTLEV_INT1) 
{
	uint8_t c1, c2, i;
	uint8_t channel_encoding, frame_number;
	uint16_t sw;
	uint8_t channel;
	uint32_t now = time_keeper_get_time_ticks() ;

	// If byte received
	if (REMOTE_UART.csr & AVR32_USART_CSR_RXRDY_MASK) 
	{
		uint32_t dt_interrupt = now - sat.last_interrupt;
		sat.last_interrupt = now;

		// Clear buffer if it contains too old data
		if ( dt_interrupt > 2500 ) 
		{
			buffer_clear(&sat.receiver);
			// LED_Toggle(LED2);
		}

		// Add new byte to buffer
		c1 = (uint8_t)REMOTE_UART.rhr;
		buffer_put(&sat.receiver, c1);
		
		// If frame is complete, decode channels
		if ( buffer_bytes_available(&sat.receiver) == 16 ) 
		{
			// first two bytes are status info
			c1 = buffer_get(&sat.receiver);
			c2 = buffer_get(&sat.receiver);
			
			//check header Bytes, otherwise discard datas
			if (c1 == 0x03 && c2 == 0xB2) 
			{
				channel_encoding = (c2 & 0x10) >> 4; 	// 0 = 11bit, 1 = 10 bit
				frame_number     = c2 & 0x03; 			// 1 = 1 frame contains all channels
			
			for (i = 0; i < 7; i++) //Max number of channels is 7 for our DSM module 
			{
				c1 = buffer_get(&sat.receiver);
				c2 = buffer_get(&sat.receiver);
				sw = (uint16_t)c1 << 8 | ((uint16_t)c2);
								
				if ( channel_encoding == 1 ) 
				{
					// highest bit is frame 0/1, bits 2-6 are channel number
					channel = ((sw >> 10))&0x0f;
					
					// 10 bits per channel
					sat.channels[channel] = ((int16_t)(sw&0x3ff) - 512) * 2;
				} 
				else if ( channel_encoding == 0 ) 
				{
					// highest bit is frame 0/1, bits 3-7 are channel number
					channel = ((sw >> 11))&0x0f;
					
						// 11 bits per channel
						sat.channels[channel] = ((int16_t)(sw&0x7ff) - 1024);
					} 
					else 
					{
						// shouldn't happen!
					}
				}	
		
				// update timing
				sat.dt 			= now - sat.last_update;
				sat.last_update = now;

				// Inidicate that new data is available
				sat.new_data_available = true;
			}
			else
			{
				buffer_clear(&sat.receiver);
				// LED_Toggle(LED2);
			}
		}
	}		
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void spektrum_satellite_init (void) 
{
	print_util_dbg_print("Reset satellite receiver \r\n");
	static const usart_options_t usart_opt = 
	{
   		.baudrate     = BAUD_REMOTE,
	    .charlength   = 8,
	    .paritytype   = USART_NO_PARITY,
	    .stopbits     = USART_1_STOPBIT,
	    .channelmode  = USART_NORMAL_CHMODE
	};
	static const gpio_map_t USART_GPIO_MAP = 
	{	
   		{ AVR32_USART1_RXD_0_1_PIN, AVR32_USART1_RXD_0_1_FUNCTION },
		{ AVR32_USART1_TXD_0_1_PIN, AVR32_USART1_TXD_0_1_FUNCTION }
	};
	
	for (int32_t i = 0; i < 16; i++) 
	{
		sat.channels[i] = 0;
		channel_center[i] = 0;
	}
	
	sat.new_data_available = false;
	 
    // Assign GPIO pins to USART_0.
    gpio_enable_module(	USART_GPIO_MAP,
                     	sizeof(USART_GPIO_MAP) / sizeof(USART_GPIO_MAP[0]) );
	
    // Initialize the USART in RS232 mode.
    usart_init_rs232( (&REMOTE_UART), &usart_opt, sysclk_get_cpu_hz() );
	INTC_register_interrupt( (__int_handler) &spectrum_handler, AVR32_USART1_IRQ, AVR32_INTC_INT1 );
	REMOTE_UART.ier = AVR32_USART_IER_RXRDY_MASK;
	
	spektrum_satellite_switch_on();
}

void spektrum_satellite_bind(void)
{
	int32_t i = 0;
	uint32_t cpu_freq = sysclk_get_cpu_hz();

	print_util_dbg_print(" \n receive bind CMD \n");
	
	// Switch off satellite
	spektrum_satellite_switch_off();
	delay_ms(100);
	spektrum_satellite_switch_on();

	// Send one first pulse
	gpio_configure_pin(DSM_RECEIVER_PIN, GPIO_DIR_INPUT | GPIO_PULL_DOWN);	
	delay_ms(1);
	gpio_configure_pin(DSM_RECEIVER_PIN, GPIO_DIR_INPUT| GPIO_INIT_LOW);
	delay_ms(10);

	// Wait for startup signal
	while ((gpio_get_pin_value(DSM_RECEIVER_PIN) == 0) && (i < 10000)) 
	{
		i++;
		delay_ms(1);
	}

	// Wait 100ms after receiver startup
	delay_ms(100);

	// create 4 pulses with 126us to set receiver to bind mode
	for (i = 0; i < 3; i++) 
	{
		gpio_configure_pin(DSM_RECEIVER_PIN, GPIO_DIR_OUTPUT | GPIO_INIT_LOW);
		cpu_delay_us(113, cpu_freq); 
		gpio_configure_pin(DSM_RECEIVER_PIN, GPIO_DIR_INPUT | GPIO_PULL_UP);	
		cpu_delay_us(118, cpu_freq);
	}
}

spektrum_satellite_t* spektrum_satellite_get_pointer(void)
{
	return &sat;
}


int16_t spektrum_satellite_get_channel(uint8_t index) 
{
	return sat.channels[index];
}


int16_t spektrum_satellite_get_neutral(uint8_t index) 
{
	int16_t value = spektrum_satellite_get_channel(index) - channel_center[index];

 	// clamp to dead zone
 	if ( (value > -DEADZONE) && (value < DEADZONE) )
 	{
 		value=0;
 	}

 	return value;
 }
