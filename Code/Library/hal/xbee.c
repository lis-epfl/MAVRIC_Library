/**
 * \page The MAV'RIC License
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */


/**
* \file xbee.c
*
* This file is for the xbee settings
*/


#include "xbee.h"
#include "uart_int.h"


buffer_t xbee_in_buffer;									///< The XBEE incoming buffer
byte_stream_t xbee_out_stream;								///< The XBEE outgoing byte stream
byte_stream_t xbee_in_stream;								///< The XBEE incoming byte stream

void xbee_init(int32_t UID)
{
	// uart setting
	usart_config_t usart_conf_xbee =
	{   
		.mode = UART_IN_OUT,
		.uart_device.uart = (avr32_usart_t *)&AVR32_USART0,
		.uart_device.IRQ = AVR32_USART0_IRQ,
		.uart_device.receive_stream = NULL,
		.options={
			.baudrate     = 57600,
			.charlength   = 8,
			.paritytype   = USART_NO_PARITY,
			.stopbits     = USART_1_STOPBIT,
			.channelmode  = USART_NORMAL_CHMODE },
		.rx_pin_map= {AVR32_USART0_RXD_0_0_PIN, AVR32_USART0_RXD_0_0_FUNCTION},
		.tx_pin_map= {AVR32_USART0_TXD_0_0_PIN, AVR32_USART0_TXD_0_0_FUNCTION}
	};
	uart_int_set_usart_conf(UID, &usart_conf_xbee);
	
	//uart configuration
	uart_int_init(UID);
	uart_int_register_write_stream(uart_int_get_uart_handle(UID), &(xbee_out_stream));
	// Registering streams
	buffer_make_buffered_stream_lossy(&(xbee_in_buffer), &(xbee_in_stream));
	uart_int_register_read_stream(uart_int_get_uart_handle(UID), &(xbee_in_stream));
}

byte_stream_t* xbee_get_in_stream(void)
{
	return &xbee_in_stream;
}

byte_stream_t* xbee_get_out_stream(void)
{
	return &xbee_out_stream;
}
