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
 * \file mavlink_stream.c
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief A wrapper for MAVLink to use the stream interface
 *
 ******************************************************************************/


#include "mavlink_stream.h"
#include "buffer.h"
#include "onboard_parameters.h"
#include "print_util.h"

#include "mavlink_message_handler.h"


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void mavlink_stream_init(mavlink_stream_t* mavlink_stream, const mavlink_stream_conf_t* config)
{	
	// mavlink_tx_stream                 = config->tx_stream;
	mavlink_stream->tx         		  = config->tx_stream;
	mavlink_stream->rx                = config->rx_stream;
	mavlink_stream->sysid             = config->sysid;
	mavlink_stream->compid            = config->compid;
	mavlink_stream->use_dma			  = config->use_dma;
	mavlink_stream->msg_available     = false;
	
//	mavlink_system.sysid              = config->sysid;  // System ID, 1-255
//	mavlink_system.compid             = config->compid; // Component/Subsystem ID, 1-255
}


void mavlink_stream_send(const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	// Copy the message to the send buffer
	uint16_t len = mavlink_msg_to_send_buffer(buf, msg);

	if( mavlink_stream->use_dma == false )
	{
		// Send byte per byte
		for (int i = 0; i < len; ++i)
		{
			mavlink_stream->tx->put(mavlink_stream->tx->data, buf[i]);
		}
	}
	else
	{
		// Use dma block transfer			// TODO: test this

		// for (int i = 0; i < len; ++i)
		// {
		// 	mavlink_stream->tx->put(mavlink_stream->tx->data, buf[i]);
		// }
		// bytestream_start_transmission(mavlink_tx_stream);		
	}
}


void mavlink_stream_receive(mavlink_stream_t* mavlink_stream) 
{
	uint8_t byte;
	byte_stream_t* stream = mavlink_stream->rx;
	mavlink_received_t* rec = &mavlink_stream->rec;

	if(mavlink_stream->msg_available == false)
	{
		while(stream->bytes_available(stream->data) > 0) 
		{
			byte = stream->get(stream->data);
			if(mavlink_parse_char(MAVLINK_COMM_0, byte, &rec->msg, &rec->status)) 
			{
				mavlink_stream->msg_available = true;
			}
		}
	}
}


void mavlink_stream_flush(mavlink_stream_t* mavlink_stream) 
{
	byte_stream_t* stream = mavlink_stream->tx;

	if (stream->flush != NULL) 
	{
		stream->flush(stream->data);	
	}
}