/*******************************************************************************
 * Copyright (c) 2009-2015, MAV'RIC Development Team
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
 * \file flow.c
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief Driver for optic flow sensors
 *
 ******************************************************************************/

#include "flow.h"


void flow_init(flow_t* flow, int32_t UID, usart_config_t usart_conf)
{
	// Init uart and buffers
	uart_int_set_usart_conf(UID, &usart_conf);
	uart_int_init(UID);
	buffer_make_buffered_stream(&(flow->uart_buffer_in), &(flow->uart_stream_in));
	buffer_make_buffered_stream(&(flow->uart_buffer_out), &(flow->uart_stream_out));
	uart_int_register_read_stream(uart_int_get_uart_handle(UID), &(flow->uart_stream_in));
	uart_int_register_write_stream(uart_int_get_uart_handle(UID), &(flow->uart_stream_out));
	
	// Init MAVLink stream
	mavlink_stream_conf_t mavlink_stream_conf =
	{
		.sysid       = 1,
		.compid      = 50,
		.use_dma     = false
	};
	mavlink_stream_init(	&(flow->mavlink_stream),
				&mavlink_stream_conf,
				&(flow->uart_stream_in),
				&(flow->uart_stream_out));
}

void flow_update(flow_t* flow)
{
	// Receive incoming bytes
	mavlink_stream_receive(&flow->mavlink_stream);

	// Check if a new message is here
	if( flow->mavlink_stream.msg_available == true )
	{
		// Get pointer to new message
		mavlink_message_t* msg = &flow->mavlink_stream.rec.msg; 
		
		// Indicate that the message was handled
		flow->mavlink_stream.msg_available = false;
		
		mavlink_optical_flow_t optical_flow_msg;

		switch( msg->msgid )
		{
			case MAVLINK_MSG_ID_OPTICAL_FLOW:
				mavlink_msg_optical_flow_decode(msg, &optical_flow_msg);
				
				flow->tmp_flow_x 	= optical_flow_msg.flow_x;
				flow->tmp_flow_x 	= optical_flow_msg.flow_y;
				flow->tmp_flow_comp_m_x = optical_flow_msg.flow_comp_m_x;
				flow->tmp_flow_comp_m_y = optical_flow_msg.flow_comp_m_y;
			break;
		}
	}
}	