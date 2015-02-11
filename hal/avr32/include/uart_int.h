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
 * \file uart_int.h
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief This file implements the UART communication protocol
 * 
 ******************************************************************************/


#ifndef UART_INT_H_
#define UART_INT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "usart.h"
#include "buffer.h"
#include "streams.h"

/**
 * \brief UART interface structure
*/
typedef struct 
{
	avr32_usart_t *uart;				///< Pointer to the AVR32 UART structure
	int32_t IRQ;						///< IRQ
	buffer_t transmit_buffer;			///< Transmission buffer
	buffer_t receive_buffer;			///< Reception buffer
	byte_stream_t *receive_stream;		///< Pointer to the reception stream
} uart_interface_t;

/**
 * \brief UART GPIO map structure
*/
typedef struct
{
	uint8_t  pin;						///< Module pin.
	uint8_t  function;					///< Module function.
} uart_gpio_map_t;


/**
 * \brief UART configuration structure
*/
typedef struct 
{
	int32_t mode;						///< UART mode
	uart_interface_t uart_device;		///< UART device to configure
	usart_options_t options;			///< UART configuration options
	uart_gpio_map_t rx_pin_map;			///< Mapping of the Rx pin
	uart_gpio_map_t tx_pin_map;			///< Mapping of the Tx pin
} usart_config_t;


/**
 * \brief Enumeration of UART mode
*/
enum UART_MODE 
{
	UART_OFF, 
	UART_IN, 
	UART_OUT, 
	UART_IN_OUT
};

/**
 * \brief Enumeration of available UARTs 
*/
enum AVAILABLE_UARTS 
{
	UART0, 
	UART1, 
	UART2, 
	UART3, 
	UART4,
	UART_COUNT
};

/**
 * \brief	Initialize the UART config
 *
 * \param	UID							The UART ID line
 * \param	usart_config				Structure with the config defined
 */
void uart_int_set_usart_conf(int32_t UID, usart_config_t* usart_config);

/**
 * \brief	Initialize the UART line
 *
 * \param	UID							The UART ID line
 */
void uart_int_init(int32_t UID);

/**
 * \brief	Get the UART line pointer
 *
 * \param	UID							The UART ID line
 *
 * \return	A pointer to the UART 
 */
usart_config_t *uart_int_get_uart_handle(int32_t UID);

/**
 * \brief	Blocking operation to retrieve a received byte from uart
 * \param	usart_conf	The pointer to the UART receive buffer
 *
 * \return	The byte received on the UART
 */
int8_t  uart_int_get_byte(usart_config_t *usart_conf);

/**
 * \brief returns number of received bytes in the receive buffer
 *
 * \param	usart_conf	The pointer to the UART line
 *
 * \return	the number of received bytes in the receive buffer
 */
int32_t uart_int_bytes_available(usart_config_t *usart_conf);

/**
 * \brief Non-blocking operation to append a byte to the uart send buffer if buffer is full, the command has no effect  (returns -1).
 *
 * \param	usart_conf	The pointer to the UART line
 * \param	data		The data to be added to the UART buffer
 *
 * \return	TODO: clean this up, should be a void function
 */
void uart_int_send_byte(usart_config_t *usart_conf, uint8_t data);

/** 
 * \brief	Blocking operation to flush the uart buffer. Returns once the last byte has been passed to hardware for transmission.
 *
 * \param	usart_conf	The pointer to the UART line to be flushed
 */
void uart_int_flush(usart_config_t *usart_conf );

/**
 * \brief Registers a stream interface with the UART transmitter (data put into the stream will be sent). 
 *
 * This function will create a blocking
 * interface - if the output buffer is full, the function will block
 * until there is space.
 * 
 * \param	usart_conf	The pointer to the UART line
 * \param	stream		The pointer to the stream on which you want to register a write stream
 */
void uart_int_register_write_stream(usart_config_t *usart_conf, byte_stream_t *stream);

/**
 * \brief Registers a stream interface with the UART transmitter (data put into the stream will be sent). 
 * This function will create a non-blocking interface. If the output buffer is full, the function will
 * overwrite the oldest data in the buffer.
 *
 * \param	usart_conf	The pointer to the UART line
 * \param	stream		The pointer to the stream on which you want to register a write stream
 */
void uart_int_register_write_stream_nonblocking(usart_config_t *usart_conf, byte_stream_t *stream);

/**
 * \brief Registers a stream interface with the UART receiver (data from the stream will be read). 
 * 
 * \param	usart_conf	The pointer to the UART line
 * \param	stream		The pointer to the stream on which you want to register a read stream
 */
void uart_int_register_read_stream(usart_config_t *usart_conf,  byte_stream_t *stream);

#ifdef __cplusplus
}
#endif

#endif /* UART_INT_H_ */
