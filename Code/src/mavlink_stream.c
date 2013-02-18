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
	uint8_t max_param_name_length;
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
	
	global_data.max_param_name_length = 10;
}

void handle_mavlink_message(Mavlink_Received_t* rec) {
	switch(rec->msg.msgid) {
		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
			// Send parameters
			/* 
			TODO : if the number of parameters becomes large, 
			it would be better to send the values at a lower rate to reduce bandwidth footprint
			*/
			for (uint16_t i = 0; i < ONBOARD_PARAM_COUNT; i++) {
				mavlink_msg_param_value_send(MAVLINK_COMM_0,
												(int8_t*)global_data.param_name[i],
												global_data.param[i],
												MAVLINK_TYPE_FLOAT,
												ONBOARD_PARAM_COUNT,
												i);
			}
		}
		break;
		case MAVLINK_MSG_ID_PARAM_SET: {
			mavlink_param_set_t set;
			mavlink_msg_param_set_decode(&rec->msg, &set);
 
			// Check if this message is for this system
			if ((uint8_t) set.target_system == (uint8_t) mavlink_system.sysid) {
				char* key = (char*) set.param_id;
				
				for (uint16_t i = 0; i < ONBOARD_PARAM_COUNT; i++) {
					bool match = true;
					for (uint16_t j = 0; j < global_data.max_param_name_length; j++) {
						// Compare
						if ((char)global_data.param_name[i][j] != (char)key[j]) {
							match = false;
						}
 
						// End matching if null termination is reached
						if (((char) global_data.param_name[i][j]) == '\0') {
							break;
						}
					}
 
					// Check if matched
					if (match) {
						// Only write and emit changes if there is actually a difference
						if (global_data.param[i] != set.param_value && set.param_type == MAVLINK_TYPE_FLOAT) {
							global_data.param[i] = set.param_value;
							// Report back new value
							mavlink_msg_param_value_send(MAVLINK_COMM_0,
														(int8_t*) global_data.param_name[i],
														global_data.param[i], MAVLINK_TYPE_FLOAT, 
														ONBOARD_PARAM_COUNT, i);
						}
					}
				}
			}
		}
		break;
		/* 
		TODO : add other cases
		*/
	}
}