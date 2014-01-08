/*! \file *********************************************************************
 *
 * \brief USART Serial configuration
 *
 * Copyright (C) 2011 Atmel Corporation. All rights reserved.
 *
 * \page License
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
 * 3. The name of Atmel may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 * Atmel AVR product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */
 
#ifndef CONF_USART_SERIAL_H_INCLUDED
#define CONF_USART_SERIAL_H_INCLUDED

#include "usart.h"
#include "gpio.h"
#include "buffer.h"
#include "streams.h"


#define XBEE_UART_ID 0
#define DEBUG_UART_ID 4

enum UART_MODE {UART_OFF, UART_IN, UART_OUT, UART_IN_OUT};

enum AVAILABLE_UARTS {UART0, UART1, UART2, UART3, UART4, 
					 UART_COUNT};

typedef struct {
	avr32_usart_t *uart;
	int IRQ;
	Buffer_t transmit_buffer;
	Buffer_t receive_buffer;
	byte_stream_t *receive_stream;
} uart_interface_t;


typedef struct
{
	unsigned char pin;              //!< Module pin.
	unsigned char function;         //!< Module function.
} uart_gpio_map_t;


typedef struct {
	int mode;
	uart_interface_t uart_device; 
	usart_options_t options; 
	uart_gpio_map_t rx_pin_map;
	uart_gpio_map_t tx_pin_map;
	} usart_config_t;

// USART options.
static volatile usart_config_t usart_opt[UART_COUNT] =
{	
	{   .mode=UART_IN_OUT,
		.uart_device.uart=&AVR32_USART0, 
		.uart_device.IRQ=AVR32_USART0_IRQ,
		.uart_device.receive_stream=NULL,
		.options={
		 .baudrate     = 57600,
		 .charlength   = 8,
		 .paritytype   = USART_NO_PARITY,
		 .stopbits     = USART_1_STOPBIT,
		 .channelmode  = USART_NORMAL_CHMODE },
		 .rx_pin_map= {AVR32_USART0_RXD_0_0_PIN, AVR32_USART0_RXD_0_0_FUNCTION},
		 .tx_pin_map= {AVR32_USART0_TXD_0_0_PIN, AVR32_USART0_TXD_0_0_FUNCTION}
	}, 
	{   .mode=UART_OFF,
		.uart_device.uart=&AVR32_USART1,
		.uart_device.IRQ=AVR32_USART1_IRQ,
		.uart_device.receive_stream=NULL,
		.options={
			.baudrate     = 57600,
			.charlength   = 8,
			.paritytype   = USART_NO_PARITY,
			.stopbits     = USART_1_STOPBIT,
			.channelmode  = USART_NORMAL_CHMODE },
			.rx_pin_map= {AVR32_USART1_RXD_1_PIN, AVR32_USART1_RXD_1_FUNCTION},
			.tx_pin_map= {AVR32_USART1_TXD_1_PIN, AVR32_USART1_TXD_1_FUNCTION}
	},
	{   .mode=UART_OFF,
		.uart_device.uart=&AVR32_USART2,
		.uart_device.IRQ=AVR32_USART2_IRQ,
		.uart_device.receive_stream=NULL,
		.options={
			.baudrate     = 57600,
			.charlength   = 8,
			.paritytype   = USART_NO_PARITY,
			.stopbits     = USART_1_STOPBIT,
			.channelmode  = USART_NORMAL_CHMODE },
			.rx_pin_map= {AVR32_USART2_RXD_0_0_PIN, AVR32_USART2_RXD_0_0_FUNCTION},
			.tx_pin_map= {AVR32_USART2_TXD_0_0_PIN, AVR32_USART2_TXD_0_0_FUNCTION}
	},
	{   .mode=UART_OFF,
		.uart_device.uart=&AVR32_USART3,
		.uart_device.IRQ=AVR32_USART3_IRQ,
		.uart_device.receive_stream=NULL,
		.options={
			.baudrate     = 38400,
			.charlength   = 8,
			.paritytype   = USART_NO_PARITY,
			.stopbits     = USART_1_STOPBIT,
			.channelmode  = USART_NORMAL_CHMODE },
			.rx_pin_map= {AVR32_USART3_RXD_2_PIN, AVR32_USART3_RXD_2_FUNCTION},
			.tx_pin_map= {AVR32_USART3_TXD_2_PIN, AVR32_USART3_TXD_2_FUNCTION}
// 			.rx_pin_map= {AVR32_USART3_RXD_0_0_PIN, AVR32_USART3_RXD_0_0_FUNCTION},
// 			.tx_pin_map= {AVR32_USART3_TXD_0_0_PIN, AVR32_USART3_TXD_0_0_FUNCTION}
	},
	{   .mode=UART_IN_OUT,
		.uart_device=&AVR32_USART4,
		.uart_device.IRQ=AVR32_USART4_IRQ,
		.uart_device.receive_stream=NULL,
		.options={
			.baudrate     = 115200,
			.charlength   = 8,
			.paritytype   = USART_NO_PARITY,
			.stopbits     = USART_1_STOPBIT,
			.channelmode  = USART_NORMAL_CHMODE },
			.rx_pin_map= {AVR32_USART4_RXD_2_PIN, AVR32_USART4_RXD_2_FUNCTION},
			.tx_pin_map= {AVR32_USART4_TXD_2_PIN, AVR32_USART4_TXD_2_FUNCTION}
	}
};



#endif /* CONF_USART_SERIAL_H_INCLUDED */
