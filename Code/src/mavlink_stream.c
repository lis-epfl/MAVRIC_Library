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

#define ONBOARD_PARAM_COUNT 3

struct global_struct
{
	float param[ONBOARD_PARAM_COUNT];
	char param_name[ONBOARD_PARAM_COUNT][MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN];
};

struct global_struct global_data;

void global_data_reset_param_defaults(void)
{
	global_data.param[0] = 1;
	strcpy(global_data.param_name[0], "PID_P_GAIN");

	global_data.param[1] = 0.345;
	strcpy(global_data.param_name[1], "PID_I_GAIN");
	
	global_data.param[2] = 1.5;
	strcpy(global_data.param_name[2], "PID_D_GAIN");
}

void handle_mavlink_message(Mavlink_Received_t* rec) {
	switch(rec->msg.msgid) {
		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
			// Send parameters
			uint8_t param_i = 0;
			while(param_i < ONBOARD_PARAM_COUNT) {
				mavlink_msg_param_value_send(MAVLINK_COMM_0,
												(int8_t*)global_data.param_name[param_i],
												global_data.param[param_i],
												MAVLINK_TYPE_FLOAT,
												ONBOARD_PARAM_COUNT,
												param_i);
				param_i++;
			}
			/*
			mavlink_msg_param_value_send(MAVLINK_COMM_0,
								(int8_t*)"PID_D_GAIN",
								123,
								MAVLINK_TYPE_FLOAT,
								ONBOARD_PARAM_COUNT,
								param_i);*/
		}
		break;
		case MAVLINK_MSG_ID_PARAM_SET: {
			// Update parameters
		}
		break;
		
		/* 
		TODO : add other cases
		*/
	}
}