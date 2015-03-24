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
 * \file uart_int.c
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief This file implements the UART communication protocol
 * 
 ******************************************************************************/


#include "uart_int.h"
#include "buffer.h"
#include "gpio.h"
#include "streams.h"
#include "sysclk.h"


static usart_config_t usart_conf[UART_COUNT];			///< Declaration of the object to store UARTs configuration

// macro for interrupt handler
#define UART_HANDLER(UID) ISR(uart_handler_##UID, usart_conf[UID].uart_device.IRQ, AVR32_INTC_INTLEV_INT1) {\
	uint8_t c1;\
	int32_t csr = usart_conf[UID].uart_device.uart->csr;\
	if (csr & AVR32_USART_CSR_RXRDY_MASK) {\
		c1 = (uint8_t)usart_conf[UID].uart_device.uart->rhr;\
		if (usart_conf[UID].uart_device.receive_stream == NULL) {\
			buffer_put_lossy(&(usart_conf[UID].uart_device.receive_buffer), c1);\
		} else {\
			usart_conf[UID].uart_device.receive_stream->put(usart_conf[UID].uart_device.receive_stream->data, c1);\
		}\
	}\
	if (csr & AVR32_USART_CSR_TXRDY_MASK) {\
		if (buffer_bytes_available(&(usart_conf[UID].uart_device.transmit_buffer)) > 0) {\
			c1 = buffer_get(&(usart_conf[UID].uart_device.transmit_buffer));\
			usart_conf[UID].uart_device.uart->thr = c1;\
		}\
		if (buffer_bytes_available(&(usart_conf[UID].uart_device.transmit_buffer)) == 0) {\
				usart_conf[UID].uart_device.uart->idr = AVR32_USART_IDR_TXRDY_MASK;\
		}\
	}\
}			
		

// define interrupt handlers using above macro
UART_HANDLER(0);
UART_HANDLER(1);
UART_HANDLER(2);
UART_HANDLER(3);
UART_HANDLER(4);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief Register UART handler
 *
 * \param	UID		UART identification number
 */
void register_UART_handler(int32_t UID);

/**
 * \brief Empty UART out buffer
 *
 * \param	usart_conf	Pointer to the usart configuration structure
 */
int32_t uart_out_buffer_empty(usart_config_t *usart_conf);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void register_UART_handler(int32_t UID)
{
	switch(UID)
	{
		case 0: 	
			INTC_register_interrupt( (__int_handler) &uart_handler_0, AVR32_USART0_IRQ, AVR32_INTC_INT1);
			break;
		case 1: 
			INTC_register_interrupt( (__int_handler) &uart_handler_1, usart_conf[1].uart_device.IRQ, AVR32_INTC_INT1);
			break;
		case 2: 
			INTC_register_interrupt( (__int_handler) &uart_handler_2, usart_conf[2].uart_device.IRQ, AVR32_INTC_INT1);
			break;
		case 3: 
			INTC_register_interrupt( (__int_handler) &uart_handler_3, usart_conf[3].uart_device.IRQ, AVR32_INTC_INT1);
			break;
		case 4: 
			INTC_register_interrupt( (__int_handler) &uart_handler_4, usart_conf[4].uart_device.IRQ, AVR32_INTC_INT1);
			break;
	}
}


int32_t uart_out_buffer_empty(usart_config_t *usart_conf)
{
	return buffer_empty(&(usart_conf->uart_device.transmit_buffer));
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------


void uart_int_set_usart_conf(int32_t UID, usart_config_t* usart_config)
{
	usart_conf[UID].mode						= usart_config->mode;
	usart_conf[UID].uart_device.uart			= usart_config->uart_device.uart;
	usart_conf[UID].uart_device.IRQ				= usart_config->uart_device.IRQ;
	usart_conf[UID].uart_device.receive_stream	= usart_config->uart_device.receive_stream;
	usart_conf[UID].options.baudrate			= usart_config->options.baudrate;
	usart_conf[UID].options.charlength			= usart_config->options.charlength;
	usart_conf[UID].options.paritytype			= usart_config->options.paritytype;
	usart_conf[UID].options.stopbits			= usart_config->options.stopbits;
	usart_conf[UID].options.channelmode			= usart_config->options.channelmode; 		
	usart_conf[UID].rx_pin_map					= usart_config->rx_pin_map;
	usart_conf[UID].tx_pin_map					= usart_config->tx_pin_map;
}

void uart_int_init(int32_t UID) 
{
	if ((usart_conf[UID].mode&UART_IN) > 0)
	{
		gpio_enable_module_pin(usart_conf[UID].rx_pin_map.pin, usart_conf[UID].rx_pin_map.function); 
	}
	
	if ((usart_conf[UID].mode&UART_OUT) > 0)
	{
		gpio_enable_module_pin(usart_conf[UID].tx_pin_map.pin, usart_conf[UID].tx_pin_map.function); 
	}

	usart_init_rs232( usart_conf[UID].uart_device.uart, &(usart_conf[UID].options), sysclk_get_cpu_hz()); 
	//usart_write_line(usart_conf[UID].uart_device.uart, "UART initialised\r\n");
	
	register_UART_handler(UID);
	
	buffer_init(&(usart_conf[UID].uart_device.transmit_buffer));
	buffer_init(&(usart_conf[UID].uart_device.receive_buffer));
	
	if (((usart_conf[UID].mode)&UART_IN) > 0)
	{
		usart_conf[UID].uart_device.uart->ier = AVR32_USART_IER_RXRDY_MASK;
	}
	
	//if (usart_conf[UID].mode&UART_OUT > 0)
	//{
		//usart_conf[UID].uart_device.uart->ier = AVR32_USART_IER_TXRDY_MASK;
	//}
} 

usart_config_t *uart_int_get_uart_handle(int32_t UID) 
{
	return &usart_conf[UID];
}

int8_t  uart_int_get_byte(usart_config_t *usart_conf) 
{
	return buffer_get(&(usart_conf->uart_device.receive_buffer));
}

int32_t uart_int_bytes_available(usart_config_t *usart_conf) 
{
	return buffer_bytes_available(&(usart_conf->uart_device.receive_buffer));
}

void uart_int_send_byte(usart_config_t *usart_conf, uint8_t data) 
{
//	usart_write_line(usart_conf->uart_device.uart, "\ns");
	while (buffer_put(&(usart_conf->uart_device.transmit_buffer), data) < 0);
	if ((buffer_bytes_available(&(usart_conf->uart_device.transmit_buffer)) >= 1))//&&
//	  (usart_conf->uart_device.uart->csr & AVR32_USART_CSR_TXRDY_MASK)) 
	{ // if there is exactly one byte in the buffer (this one...), and transmitter ready
		 // kick-start transmission
//		usart_conf->uart_device.uart->thr='c';//buffer_get(&(usart_conf->uart_device.transmit_buffer));
		usart_conf->uart_device.uart->ier = AVR32_USART_IER_TXRDY_MASK;
	}		
}

void uart_int_flush(usart_config_t *usart_conf) 
{
	usart_conf->uart_device.uart->ier = AVR32_USART_IER_TXRDY_MASK;
	while (!buffer_empty(&(usart_conf->uart_device.transmit_buffer)));
}

void uart_int_register_write_stream(usart_config_t *usart_conf, byte_stream_t *stream) 
{
	stream->get = NULL;
	//stream->get = &uart_int_get_byte;
	stream->put = (uint8_t(*)(stream_data_t*, uint8_t))&uart_int_send_byte;			// Here we need to explicitly cast the function to match the prototype
	stream->flush = (void(*)(stream_data_t*))&uart_int_flush;						// stream->get and stream->put expect stream_data_t* as first argument
	stream->buffer_empty = (int32_t(*)(stream_data_t*))&uart_out_buffer_empty;			// but buffer_get and buffer_put take buffer_t* as first argument
	stream->data = usart_conf;
}

void uart_int_register_read_stream(usart_config_t *usart_conf,  byte_stream_t *stream) 
{
	usart_conf->uart_device.receive_stream = stream;
}


	

