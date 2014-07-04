/** 
 * \page The MAV'RIC license
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */
 

/**
 * \file uart_int.h
 *
 * This file implments the UART communication protocol
 */


#ifndef UART_INT_H_
#define UART_INT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "compiler.h"
#include "buffer.h"
#include "conf_usart_serial.h"
#include "streams.h"

/**
 * \brief	Initialize the UART line
 *
 * \param	UID							The UART ID line
 */
void uart_int_init(int UID);

/**
 * \brief	Get the UART line pointer
 *
 * \param	UID							The UART ID line
 *
 * \return	A pointer to the UART 
 */
usart_config_t *uart_int_get_uart_handle(int UID);

/**
 * \brief	Blocking operation to retrieve a received byte from uart
 * \param	usart_opt	The pointer to the UART receive buffer
 *
 * \return	The byte received on the UART
 */
char uart_int_get_byte(usart_config_t *usart_opt);

/**
 * \brief returns number of received bytes in the receive buffer
 *
 * \param	usart_opt	The pointer to the UART line
 *
 * \return	the number of received bytes in the receive buffer
 */
int uart_int_bytes_available(usart_config_t *usart_opt);

/**
 * \brief Non-blocking operation to append a byte to the uart send buffer if buffer is full, the command has no effect  (returns -1).
 *
 * \param	usart_opt	The pointer to the UART line
 * \param	data		The data to be added to the UART buffer
 *
 * \return	TODO: clean this up, should be a void function
 */
uint8_t uart_int_send_byte(usart_config_t *usart_opt, char data);

/** 
 * \brief	Blocking operation to flush the uart buffer. Returns once the last byte has been passed to hardware for transmission.
 *
 * \param	usart_opt	The pointer to the UART line to be flushed
 */
void uart_int_flush(usart_config_t *usart_opt );

/**
 * \brief Registers a stream interface with the UART transmitter (data put into the stream will be sent). 
 *
 * This function will create a blocking
 * interface - if the output buffer is full, the function will block
 * until there is space.
 * 
 * \param	usart_opt	The pointer to the UART line
 * \param	stream		The pointer to the stream on which you want to register a write stream
 */
void uart_int_register_write_stream(usart_config_t *usart_opt, byte_stream_t *stream);

/**
 * \brief Registers a stream interface with the UART transmitter (data put into the stream will be sent). 
 * This function will create a non-blocking interface. If the output buffer is full, the function will
 * overwrite the oldest data in the buffer.
 *
 * \param	usart_opt	The pointer to the UART line
 * \param	stream		The pointer to the stream on which you want to register a write stream
 */
void uart_int_register_write_stream_nonblocking(usart_config_t *usart_opt, byte_stream_t *stream);

void uart_int_register_read_stream(usart_config_t *usart_opt,  byte_stream_t *stream);

#ifdef __cplusplus
}
#endif

#endif /* UART_INT_H_ */
