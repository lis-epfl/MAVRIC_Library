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
#include "time_keeper.h"


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

/**
 * \brief Swap bytes iof signed 16 bits integer
 */
static inline int16_t endian_rev16(int16_t data)
{
	return ((data >> 8) & 0x00ff) | ((data & 0x00ff) << 8);
};


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool flow_init(flow_t* flow, int32_t UID, usart_config_t usart_conf)
{
	bool success = true;

	// Init members
	flow->last_update_us  = time_keeper_get_micros();
	flow->handshake_state = FLOW_NO_HANDSHAKE; 
	flow->of_count 	      = 0;

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
		.debug       = true,
	};
	success &= mavlink_stream_init(	&(flow->mavlink_stream),
					&mavlink_stream_conf,
					&(flow->uart_stream_in),
					&(flow->uart_stream_out));

	return success;
}


bool flow_update(flow_t* flow)
{
	// Update time
	flow->last_update_us  = time_keeper_get_micros();

	// Receive incoming bytes
	mavlink_stream_receive(&flow->mavlink_stream);

	// Check if a new message is here
	if( flow->mavlink_stream.msg_available == true )
	{
		// Get pointer to new message
		mavlink_message_t* msg = &flow->mavlink_stream.rec.msg; 
		
		// Indicate that the message was handled
		flow->mavlink_stream.msg_available = false;
		
		// declare messages
		mavlink_optical_flow_t 			optical_flow_msg;
		mavlink_data_transmission_handshake_t 	handshake_msg;
		mavlink_encapsulated_data_t		data_msg;

		switch( msg->msgid )
		{
			case MAVLINK_MSG_ID_OPTICAL_FLOW:
				// Decode message
				mavlink_msg_optical_flow_decode(msg, &optical_flow_msg);
				
				flow->tmp_flow_x 	= optical_flow_msg.flow_x;
				flow->tmp_flow_y 	= optical_flow_msg.flow_y;
				flow->tmp_flow_comp_m_x = optical_flow_msg.flow_comp_m_x;
				flow->tmp_flow_comp_m_y = optical_flow_msg.flow_comp_m_y;
			break;

			case MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE:
				// Decode message
				mavlink_msg_data_transmission_handshake_decode(msg, &handshake_msg);
				
				// Get type of handshake
				flow->handshake_state = handshake_msg.jpg_quality;
				
				// Get number of of vectors
				flow->of_count 	      = handshake_msg.width;
				if( flow->of_count > 125 )
				{
					flow->of_count = 125;
				}

				// Get total payload size
				flow->n_packets	      = handshake_msg.packets;
				flow->size_data	      = handshake_msg.size;
				if( flow->size_data > 500 )
				{
					flow->size_data = 500;
				}
			break;

			case MAVLINK_MSG_ID_ENCAPSULATED_DATA:
				// Decode message
				mavlink_msg_encapsulated_data_decode(msg, &data_msg);

				// Handle according to hanshake state
				switch( flow->handshake_state )
				{
					case FLOW_HANDSHAKE_METADATA:
						if( data_msg.seqnr < flow->n_packets - 1 )
						{
							// not last packet
							for ( int i = 0; i < MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN; ++i)
							{
								flow->of_loc.data[i + data_msg.seqnr * MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = data_msg.data[i];
							}
						}
						else if( data_msg.seqnr < flow->n_packets )
						{
							// last packet
							for ( int i = 0; i < flow->size_data; ++i)
							{
								flow->of_loc.data[i + data_msg.seqnr * MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = data_msg.data[i];
							}

							// swap bytes
							for (int i = 0; i < flow->of_count; ++i)
							{
								flow->of_loc.x[i] = endian_rev16(flow->of_loc.x[i]);
								flow->of_loc.y[i] = endian_rev16(flow->of_loc.y[i]);
							}
						}
					break;

					default:
						if( data_msg.seqnr < (flow->n_packets - 1) )
						{
							// not last packet
							for ( int i = 0; i < MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN; ++i)
							{
								flow->of.data[i + data_msg.seqnr * MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = data_msg.data[i];
							}
						}
						else if( data_msg.seqnr == (flow->n_packets-1) )
						{
							// last packet
							for ( int i = 0; i < flow->size_data % MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN; ++i)
							{
								flow->of.data[i + data_msg.seqnr * MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = data_msg.data[i];
							}

							// swap bytes
							// for (int i = 0; i < flow->of_count; ++i)
							// for (int i = 0; i < 125; ++i)
							// {
								// flow->of.x[i] = endian_rev16(flow->of.x[i]);
								// flow->of.y[i] = endian_rev16(flow->of.y[i]);
							// }
						}
					break;
				}
			break;
			
			default:
			break;
		}
	}

	return true;
}	