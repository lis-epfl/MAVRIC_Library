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


static inline usart_config_t usart_default_config_console()
{
    usart_config_t usart_config             = {};
    usart_config.mode                       = UART_IN_OUT;
    usart_config.uart_device.uart           = (avr32_usart_t *)&AVR32_USART4;
    usart_config.uart_device.IRQ            = AVR32_USART4_IRQ;
    usart_config.uart_device.receive_stream = NULL;
    usart_config.options                    = {};
    usart_config.options.baudrate           = 57600;
    usart_config.options.charlength         = 8;
    usart_config.options.paritytype         = USART_NO_PARITY;
    usart_config.options.stopbits           = USART_1_STOPBIT;
    usart_config.options.channelmode        = USART_NORMAL_CHMODE;
    usart_config.rx_pin_map                 = {AVR32_USART4_RXD_2_PIN, AVR32_USART4_RXD_2_FUNCTION};
    usart_config.tx_pin_map                 = {AVR32_USART4_TXD_2_PIN, AVR32_USART4_TXD_2_FUNCTION};

    return usart_config;
};


static inline usart_config_t usart_default_config_gps()
{
    usart_config_t usart_config             = {};
    usart_config.mode						= UART_IN_OUT;
    usart_config.uart_device.uart			= (avr32_usart_t *)&AVR32_USART3;
    usart_config.uart_device.IRQ			= AVR32_USART3_IRQ;
    usart_config.uart_device.receive_stream	= NULL;
    usart_config.options    				= {};
	usart_config.options.baudrate      		= 38400;
	usart_config.options.charlength			= 8;
	usart_config.options.paritytype			= USART_NO_PARITY;
	usart_config.options.stopbits			= USART_1_STOPBIT;
	usart_config.options.channelmode		= USART_NORMAL_CHMODE;
    usart_config.rx_pin_map					= {AVR32_USART3_RXD_0_0_PIN, AVR32_USART3_RXD_0_0_FUNCTION};
    usart_config.tx_pin_map					= {AVR32_USART3_TXD_0_0_PIN, AVR32_USART3_TXD_0_0_FUNCTION};

    return usart_config;
};


static inline usart_config_t usart_default_config_spektrum()
{
    usart_config_t usart_config             = {};
    usart_config.mode						= UART_IN_OUT;
    usart_config.uart_device.uart			= (avr32_usart_t *)&AVR32_USART1;
    usart_config.uart_device.IRQ			= AVR32_USART1_IRQ;
    usart_config.uart_device.receive_stream	= NULL;
    usart_config.options                    = {};
	usart_config.options.baudrate      		= 115200;
	usart_config.options.charlength			= 8;
	usart_config.options.paritytype			= USART_NO_PARITY;
	usart_config.options.stopbits			= USART_1_STOPBIT;
	usart_config.options.channelmode		= USART_NORMAL_CHMODE;
    usart_config.rx_pin_map					= { AVR32_USART1_RXD_0_1_PIN, AVR32_USART1_RXD_0_1_FUNCTION };
    usart_config.tx_pin_map					= { AVR32_USART1_TXD_0_1_PIN, AVR32_USART1_TXD_0_1_FUNCTION };

    return usart_config;
};


static inline usart_config_t usart_default_config_xbee()
{
    usart_config_t usart_config             = {};
    usart_config.mode						= UART_IN_OUT;
    usart_config.uart_device.uart			= (avr32_usart_t *)&AVR32_USART0;
    usart_config.uart_device.IRQ			= AVR32_USART0_IRQ;
    usart_config.uart_device.receive_stream = NULL;
    usart_config.options					= {};
	usart_config.options.baudrate      		= 57600;
	usart_config.options.charlength			= 8;
	usart_config.options.paritytype			= USART_NO_PARITY;
	usart_config.options.stopbits			= USART_1_STOPBIT;
	usart_config.options.channelmode		= USART_NORMAL_CHMODE;
    usart_config.rx_pin_map					= {AVR32_USART0_RXD_0_0_PIN, AVR32_USART0_RXD_0_0_FUNCTION};
    usart_config.tx_pin_map					= {AVR32_USART0_TXD_0_0_PIN, AVR32_USART0_TXD_0_0_FUNCTION};

    return usart_config;
};


#ifdef __cplusplus
}
#endif

#endif // UART_DEFAULT_CONFIG_H_