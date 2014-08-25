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
 * \file console.h
 *
 * This file configures the console UART communication
 */


#ifndef CONSOLE_H_
#define CONSOLE_H_

#include "streams.h"
#include "buffer.h"
#include "uart_int.h"
#include "usb_int.h"

typedef enum 
{
	CONSOLE_UART0 = UART0,
	CONSOLE_UART1 = UART1,
	CONSOLE_UART2 = UART2,
	CONSOLE_UART3 = UART3,
	CONSOLE_UART4 = UART4,
	CONSOLE_USB   = USB
}console_port_t;

/**
 * \brief				Initialize the console module
 *
 * \param console_port	UART or USB device number, from UART0 to UART4 for UART, otherwise USB
 */
void console_init(console_port_t console_port);

/**
 * \brief				Return the console in stream
 *
 * \return				the pointer to the console in stream
 */
byte_stream_t* console_get_in_stream(void);

/**
 * \brief				Return the console out stream
 *
 * \return				the pointer to the console out stream
 */
byte_stream_t* console_get_out_stream(void);

#endif /* CONSOLE_H_ */