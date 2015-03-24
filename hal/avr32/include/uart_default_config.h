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
 * \file uart_default_config.h
 * 
 * \author MAV'RIC Team
 *   
 * \brief Default configuration for uarts
 *
 ******************************************************************************/

#ifndef UART_DEFAULT_CONFIG_H_
#define UART_DEFAULT_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "uart_int.h"

static const usart_config_t usart_default_config_console =
{
    .mode                       = UART_IN_OUT,
    .uart_device.uart           = (avr32_usart_t *)&AVR32_USART4,
    .uart_device.IRQ            = AVR32_USART4_IRQ,
    .uart_device.receive_stream = NULL,
    .options = 
    {
        .baudrate       = 57600,
        .charlength     = 8,
        .paritytype     = USART_NO_PARITY,
        .stopbits       = USART_1_STOPBIT,
        .channelmode    = USART_NORMAL_CHMODE 
    },
    .rx_pin_map         = {AVR32_USART4_RXD_2_PIN, AVR32_USART4_RXD_2_FUNCTION},
    .tx_pin_map         = {AVR32_USART4_TXD_2_PIN, AVR32_USART4_TXD_2_FUNCTION}
};


static const usart_config_t usart_default_config_gps =
{
    .mode						= UART_IN_OUT,
    .uart_device.uart			= (avr32_usart_t *)&AVR32_USART3,
    .uart_device.IRQ			= AVR32_USART3_IRQ,
    .uart_device.receive_stream	= NULL,
    .options					=
	{
		.baudrate				= 38400,
		.charlength				= 8,
		.paritytype				= USART_NO_PARITY,
		.stopbits				= USART_1_STOPBIT,
		.channelmode			= USART_NORMAL_CHMODE 
	},
    .rx_pin_map					= {AVR32_USART3_RXD_0_0_PIN, AVR32_USART3_RXD_0_0_FUNCTION},
    .tx_pin_map					= {AVR32_USART3_TXD_0_0_PIN, AVR32_USART3_TXD_0_0_FUNCTION}
};


static const usart_config_t usart_default_config_spektrum =
{
    .mode						= UART_IN_OUT,
    .uart_device.uart			= (avr32_usart_t *)&AVR32_USART1,
    .uart_device.IRQ			= AVR32_USART1_IRQ,
    .uart_device.receive_stream	= NULL,
    .options					=
	{
		.baudrate				= 115200,
		.charlength				= 8,
		.paritytype				= USART_NO_PARITY,
		.stopbits				= USART_1_STOPBIT,
		.channelmode			= USART_NORMAL_CHMODE 
	},
    .rx_pin_map					= { AVR32_USART1_RXD_0_1_PIN, AVR32_USART1_RXD_0_1_FUNCTION },
    .tx_pin_map					= { AVR32_USART1_TXD_0_1_PIN, AVR32_USART1_TXD_0_1_FUNCTION }
};


static const usart_config_t usart_default_config_xbee =
{
    .mode						= UART_IN_OUT,
    .uart_device.uart			= (avr32_usart_t *)&AVR32_USART0,
    .uart_device.IRQ			= AVR32_USART0_IRQ,
    .uart_device.receive_stream = NULL,
    .options					=
	{
		.baudrate				= 57600,
		.charlength				= 8,
		.paritytype				= USART_NO_PARITY,
		.stopbits				= USART_1_STOPBIT,
		.channelmode			= USART_NORMAL_CHMODE 
	},
    .rx_pin_map					= {AVR32_USART0_RXD_0_0_PIN, AVR32_USART0_RXD_0_0_FUNCTION},
    .tx_pin_map					= {AVR32_USART0_TXD_0_0_PIN, AVR32_USART0_TXD_0_0_FUNCTION}
};


#ifdef __cplusplus
}
#endif

#endif // UART_DEFAULT_CONFIG_H_