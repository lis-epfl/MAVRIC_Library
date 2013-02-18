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

void mavlink_receive(stream_data_t* data, uint8_t element) {
	Mavlink_Received_t rec;
	if(mavlink_parse_char(MAVLINK_COMM_0, element, &rec.msg, &rec.status)) {
		handle_mavlink_message(MAVLINK_COMM_0, &rec.msg);	
	}
}

void init_mavlink(byte_stream_t *transmit_stream) {
	mavlink_system.sysid = 100; // System ID, 1-255
	mavlink_system.compid = 50; // Component/Subsystem ID, 1-255
	mavlink_out_stream=transmit_stream;
	Buffer_t *mavlink_in_buffer;
	make_buffered_stream(mavlink_in_buffer, mavlink_in_stream);
}

void handle_mavlink_message(mavlink_channel_t chan, mavlink_message_t* msg) {
	mavlink_msg_named_value_int_send(MAVLINK_COMM_0, 0, "User_val_2", 321);
}