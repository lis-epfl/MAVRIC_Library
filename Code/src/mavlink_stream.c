/*
 * mavlink_stream.c
 *
 * a wrapper for mavlink to use the stream interface
 *
 * Created: 14/02/2013 17:10:05
 *  Author: julien
 */ 

#include "mavlink_stream.h"
#include "buffer.h"


byte_stream_t* mavlink_out_stream;
byte_stream_t* mavlink_in_stream;
Buffer_t mavlink_in_buffer;

void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
	if (chan == MAVLINK_COMM_0)
	{
		//uart0_transmit(ch);
		mavlink_out_stream->put(mavlink_out_stream->data, ch);
	}
	if (chan == MAVLINK_COMM_1)
	{
		//uart1_transmit(ch);
	}
}

void init_mavlink(byte_stream_t *transmit_stream, byte_stream_t *receive_stream) {
	mavlink_system.sysid = 100; // System ID, 1-255
	mavlink_system.compid = 50; // Component/Subsystem ID, 1-255
	mavlink_out_stream = transmit_stream;
	mavlink_in_stream = receive_stream;
	make_buffered_stream(&mavlink_in_buffer, mavlink_in_stream);
}

uint8_t mavlink_receive(byte_stream_t* stream, Mavlink_Received_t* rec) {
	uint8_t byte;
	while(buffer_bytes_available(stream->data) > 0) {
		byte = stream->get(stream->data);
		if(mavlink_parse_char(MAVLINK_COMM_0, byte, &rec->msg, &rec->status)) {
			return 1;
		}
	}		
	return 0;
}

void handle_mavlink_message(Mavlink_Received_t* rec) {
	;
}