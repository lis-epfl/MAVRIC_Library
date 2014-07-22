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
 * \file mavlink_stream.c
 * 
 * A wrapper for mavlink to use the stream interface
 */


#include "mavlink_stream.h"
#include "buffer.h"
#include "onboard_parameters.h"
#include "print_util.h"

#include "mavlink_message_handler.h"


mavlink_system_t mavlink_system;
byte_stream_t* mavlink_tx_stream;


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void mavlink_comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
	if (chan == MAVLINK_COMM_0)
	{
		mavlink_tx_stream->put(mavlink_tx_stream->data, ch);
	}
}


void mavlink_stream_init(mavlink_stream_t* mavlink_stream, const mavlink_stream_conf_t* config)
{	
	mavlink_tx_stream                 = config->tx_stream;
	mavlink_stream->tx         		  = config->tx_stream;
	mavlink_stream->rx                = config->rx_stream;
	mavlink_stream->sysid             = config->sysid;
	mavlink_stream->compid            = config->compid;
	mavlink_stream->use_dma			  = config->use_dma;
	mavlink_stream->msg_available     = false;
	
	mavlink_system.sysid              = config->sysid;  // System ID, 1-255
	mavlink_system.compid             = config->compid; // Component/Subsystem ID, 1-255
}


void mavlink_stream_send(mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	// Copy the message to the send buffer
	uint16_t len = mavlink_msg_to_send_buffer(buf, msg);

	if( mavlink_stream->use_dma == false )
	{
		// Send byte per byte
		for (int i = 0; i < len; ++i)
		{
			mavlink_stream->tx->put(mavlink_tx_stream->data, buf[i]);
		}
	}
	else
	{
		// Use dma block transfer			// TODO: test this

		// for (int i = 0; i < len; ++i)
		// {
		// 	mavlink_tx_stream->put(mavlink_tx_stream->data, buf[i]);
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