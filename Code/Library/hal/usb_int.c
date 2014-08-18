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
 * \file usb_int.c
 *
 * This file implements the USB communication protocol
 */


#include "usb_int.h"
//#include "buffer.h"
//#include "gpio.h"
#include "streams.h"
//#include "sysclk.h"


static usb_config_t usb_conf;


void usb_int_set_usb_conf(usb_config_t* usb_config)
{
	usb_conf.mode						= usb_config->mode;
	usb_conf.usb_device.IRQ				= usb_config->usb_device.IRQ;
	usb_conf.usb_device.receive_stream	= usb_config->usb_device.receive_stream;
}

void usb_int_init(void)
{
	stdio_usb_init(NULL);
	
	//register_USB_handler();
	
	//buffer_init(&(usart_conf[UID].uart_device.transmit_buffer));
	//buffer_init(&(usart_conf[UID].uart_device.receive_buffer));
	
	stdio_usb_enable();
	
} 


usb_config_t *usb_int_get_usb_handle(void)
{
	return &usb_conf;
}

void usb_int_send_byte(usb_config_t *usb_conf, uint8_t data) 
{
	stdio_usb_putchar(NULL, (int)data);	
}


void usb_int_register_write_stream(usb_config_t *usb_conf, byte_stream_t *stream) 
{
	stream->get = NULL;
	stream->put = (uint8_t(*)(stream_data_t*, uint8_t))&usb_int_send_byte;			// Here we need to explicitly cast the function to match the prototype
	stream->flush = NULL;															// stream->get and stream->put expect stream_data_t* as first argument
	stream->buffer_empty = NULL;													// but buffer_get and buffer_put take Buffer_t* as first argument
	stream->data = usb_conf;
}

void  usb_int_register_read_stream(usb_config_t *usb_conf, byte_stream_t *stream)
{
	usb_conf->usb_device.receive_stream = stream;
}

