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
 * \file epuck_communication.c
 * 
 * \author MAV'RIC Team
 * \author Gregoire Heitz
 *   
 * \brief	This file configures the epuck UART communication
 *			and send remote scaled messages to the epuck to be used to drive its wheels
 *
 ******************************************************************************/


#include "epuck_communication.h"
#include "time_keeper.h"


void epuck_communication_init(epuck_communication_t* epuck_communication, const remote_t* remote, int32_t UID, usart_config_t usart_conf_epuck)
{
	//uart init
	uart_int_set_usart_conf(UID, &usart_conf_epuck);
	
	uart_int_init(UID);
	buffer_make_buffered_stream(&(epuck_communication->epuck_buffer), (epuck_communication->mavlink_stream.rx));
	uart_int_register_read_stream(uart_int_get_uart_handle(UID), (epuck_communication->mavlink_stream.rx));
	uart_int_register_write_stream(uart_int_get_uart_handle(UID), (epuck_communication->mavlink_stream.tx));
	
	//init dependencies
	epuck_communication->remote = remote;
}


task_return_t epuck_communication_update(epuck_communication_t* epuck_communication)
{
	mavlink_message_t msg;
	
	mavlink_msg_rc_channels_scaled_pack(	epuck_communication->mavlink_stream.sysid,
											epuck_communication->mavlink_stream.compid,
											&msg,
											time_keeper_get_millis(),
											0,
											epuck_communication->remote->channels[0] * 10000.0f,
											epuck_communication->remote->channels[1] * 10000.0f,
											epuck_communication->remote->channels[2] * 10000.0f,
											epuck_communication->remote->channels[3] * 10000.0f,
											epuck_communication->remote->channels[4] * 10000.0f,
											epuck_communication->remote->channels[5] * 10000.0f,
											epuck_communication->remote->channels[6] * 10000.0f,
											epuck_communication->remote->channels[7] * 10000.0f,
											epuck_communication->remote->mode.current_desired_mode.byte );
											// epuck_communication->remote->signal_quality	);
	
	mavlink_stream_send(&(epuck_communication->mavlink_stream), &msg);

	return TASK_RUN_SUCCESS;
}
