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
 * \file console.c
 *
 * This file configures the console UART communication
 */


#include "console.h"
#include "uart_int.h"
#include "usb_int.h"

buffer_t console_in_buffer;				///< The console incoming buffer
byte_stream_t console_out_stream;		///< The console outgoing byte stream
byte_stream_t console_in_stream;		///< The console incoming byte stream


void console_init(console_port_t console_port)
{
	if((console_port >= CONSOLE_UART0) && (console_port <= CONSOLE_UART4))
	{
		// uart setting
		usart_config_t usart_conf_console =
		{
			.mode=UART_IN_OUT,
			.uart_device.uart=(avr32_usart_t *)&AVR32_USART4,
			.uart_device.IRQ=AVR32_USART4_IRQ,
			.uart_device.receive_stream=NULL,
			.options={
				.baudrate     = 57600,
				.charlength   = 8,
				.paritytype   = USART_NO_PARITY,
				.stopbits     = USART_1_STOPBIT,
			.channelmode  = USART_NORMAL_CHMODE },
			.rx_pin_map= {AVR32_USART4_RXD_2_PIN, AVR32_USART4_RXD_2_FUNCTION},
			.tx_pin_map= {AVR32_USART4_TXD_2_PIN, AVR32_USART4_TXD_2_FUNCTION}
		};
		uart_int_set_usart_conf(console_port, &usart_conf_console);
		
		//uart configuration
		uart_int_init(console_port);
		uart_int_register_write_stream(uart_int_get_uart_handle(console_port), &(console_out_stream));
		// Registering streams
		buffer_make_buffered_stream_lossy(&(console_in_buffer), &(console_in_stream));
		uart_int_register_read_stream(uart_int_get_uart_handle(console_port), &(console_in_stream));
	}
	
	if(console_port == CONSOLE_USB)
	{
		// usb setting
		usb_config_t usb_conf_console =
		{
			.mode=USB_IN_OUT,
			.usb_device.IRQ=0,//AVR32_USB_IRQ,
			.usb_device.receive_stream=NULL
		};
		usb_int_set_usb_conf(&usb_conf_console);
		
		// usb configuration
		usb_int_init();
		usb_int_register_write_stream(usb_int_get_usb_handle(), &(console_out_stream));
		// Registering streams
		buffer_make_buffered_stream_lossy(&(console_in_buffer), &(console_in_stream));
		usb_int_register_read_stream(usb_int_get_usb_handle(), &(console_in_stream));
	}	
}

byte_stream_t* console_get_in_stream(void)
{
	return &console_in_stream;
}

byte_stream_t* console_get_out_stream(void)
{
	return &console_out_stream;
}
