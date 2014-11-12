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
 * \file console.h
 * 
 * \author MAV'RIC Team
 * \author Gregoire Heitz
 * \author Geraud L'Eplattenier
 *   
 * \brief This file configures the console UART communication
 *
 ******************************************************************************/


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
 * \param usart_conf_console
 * \param usb_conf_console
 */
void console_init(console_port_t console_port, usart_config_t usart_conf_console, usb_config_t usb_conf_console);

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