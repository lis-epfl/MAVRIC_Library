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
 * \file usb_int.h
 *
 * This file implements the USB communication protocol
 */


#ifndef USB_INT_H_
#define USB_INT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stdio_usb.h"
#include "buffer.h"
#include "streams.h"

typedef struct {
	int32_t IRQ;
	//Buffer_t transmit_buffer;
	//Buffer_t receive_buffer;
	byte_stream_t *receive_stream;
} usb_interface_t;

typedef struct {
	int32_t mode;
	usb_interface_t usb_device;
} usb_config_t;


enum USB_MODE 
{
	USB_OFF, 
	USB_IN, 
	USB_OUT, 
	USB_IN_OUT
};


/**
 * \brief	Initialize the USB config
 *
 * \param	usb_config					Structure with the config defined
 */
void usb_int_set_usb_conf(usb_config_t* usb_config);


/**
 * \brief	Initialize the USB line
 */
void usb_int_init(void);


/**
 * \brief	Get the USB line pointer
 *
 * \return	A pointer to the USB 
 */
usb_config_t *usb_int_get_usb_handle(void);


/**
 * \brief	Blocking operation to retrieve a received byte from uart
 * \param	usart_conf	The pointer to the UART receive buffer
 *
 * \return	The byte received on the UART
 */
//int8_t  uart_int_get_byte(usart_config_t *usart_conf);

/**
 * \brief returns number of received bytes in the receive buffer
 *
 * \param	usart_conf	The pointer to the UART line
 *
 * \return	the number of received bytes in the receive buffer
 */
//int32_t uart_int_bytes_available(usart_config_t *usart_conf);

/**
 * \brief Non-blocking operation to append a byte to the usb send buffer
 *
 * \param	usb_conf	The pointer to the USB line
 * \param	data		The data to be added to the USB buffer
 */
void usb_int_send_byte(usb_config_t *usb_conf, uint8_t data) ;


/** 
 * \brief	Blocking operation to flush the uart buffer. Returns once the last byte has been passed to hardware for transmission.
 *
 * \param	usart_conf	The pointer to the UART line to be flushed
 */
//void uart_int_flush(usart_config_t *usart_conf );

/**
 * \brief	Registers a stream interface with the USB transmitter (data put into the stream will be sent). 
 * 
 * \param	usb_conf	The pointer to the USB line
 * \param	stream		The pointer to the stream on which you want to register a write stream
 */
void usb_int_register_write_stream(usb_config_t *usb_conf, byte_stream_t *stream);

/**
 * \brief Registers a stream interface with the UART transmitter (data put into the stream will be sent). 
 * This function will create a non-blocking interface. If the output buffer is full, the function will
 * overwrite the oldest data in the buffer.
 *
 * \param	usart_conf	The pointer to the UART line
 * \param	stream		The pointer to the stream on which you want to register a write stream
 */
//void uart_int_register_write_stream_nonblocking(usart_config_t *usart_conf, byte_stream_t *stream);

/**
 * \brief Registers a stream interface with the USB receiver (data from the stream will be read). 
 * 
 * \param	usb_conf	The pointer to the USB line
 * \param	stream		The pointer to the stream on which you want to register a read stream
 */
void  usb_int_register_read_stream(usb_config_t *usb_conf, byte_stream_t *stream);

#ifdef __cplusplus
}
#endif

#endif /* USB_INT_H_ */
