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

void mavlink_receive(stream_data_t* data, uint8_t element) {
	Mavlink_Received_t rec;
	if(mavlink_parse_char(MAVLINK_COMM_0, element, &rec.msg, &rec.status)) {
		handle_mavlink_message(MAVLINK_COMM_0, &rec.msg);	
	}
}

byte_stream_t* mavlink_get_in_stream() {
	return mavlink_in_stream;	
}

void handle_mavlink_message(mavlink_channel_t chan, mavlink_message_t* msg) {
	mavlink_msg_named_value_int_send(MAVLINK_COMM_0, 0, "User_val_2", 321);
}