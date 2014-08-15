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

#include <stdint.h>
#include "usart.h"
#include "buffer.h"
#include "streams.h"

typedef struct {
avr32_usart_t *uart;
int32_t IRQ;
Buffer_t transmit_buffer;
Buffer_t receive_buffer;
byte_stream_t *receive_stream;
} uart_interface_t;


typedef struct
{
uint8_t  pin;              //!< Module pin.
uint8_t  function;         //!< Module function.
} uart_gpio_map_t;


typedef struct {
int32_t mode;
uart_interface_t uart_device;
usart_options_t options;
uart_gpio_map_t rx_pin_map;
uart_gpio_map_t tx_pin_map;
} usart_config_t;


enum UART_MODE 
{
	UART_OFF, 
	UART_IN, 
	UART_OUT, 
	UART_IN_OUT
};

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
