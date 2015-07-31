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
 * \file xbee.c
 * 
 * \author MAV'RIC Team
 * \author Gregoire Heitz
 *   
 * \brief This file is for the xbee settings
 * 
 ******************************************************************************/


#include "xbee.h"
#include "uart_int.h"


buffer_t xbee_in_buffer;									///< The XBEE incoming buffer
byte_stream_t xbee_out_stream;								///< The XBEE outgoing byte stream
byte_stream_t xbee_in_stream;								///< The XBEE incoming byte stream

void xbee_init(int32_t UID, usart_config_t usart_conf_xbee)
{
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
