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
#include "onboard_parameters.h"
#include "print_util.h"

byte_stream_t* mavlink_out_stream;
byte_stream_t* mavlink_in_stream;
Buffer_t mavlink_in_buffer;


NEW_TASK_SET (mavlink_tasks, 10)

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

void mavlink_receive_handler() {
	Mavlink_Received_t rec;
	if(mavlink_receive(mavlink_in_stream, &rec)) {
// 		dbg_print("\n Received message with ID");
// 		dbg_print_num(rec.msg.msgid, 10);
// 		dbg_print(" from system");
// 		dbg_print_num(rec.msg.sysid, 10);
// 		dbg_print( "\n");
		
		handle_mavlink_message(&rec);
	}
}

void init_mavlink(byte_stream_t *transmit_stream, byte_stream_t *receive_stream) {
	mavlink_system.sysid = 154; // System ID, 1-255
	mavlink_system.compid = 50; // Component/Subsystem ID, 1-255
	mavlink_out_stream = transmit_stream;
	mavlink_in_stream = receive_stream;
	make_buffered_stream(&mavlink_in_buffer, mavlink_in_stream);
	init_scheduler(&mavlink_tasks);
	
	register_task(&mavlink_tasks, 0, 10000, &mavlink_receive_handler);
	
}

task_return_t mavlink_protocol_update() {
	if (mavlink_out_stream->buffer_empty(mavlink_out_stream->data)) {
		run_scheduler_update(&mavlink_tasks, FIXED_PRIORITY);
	}	

}

task_set* get_mavlink_taskset() {
	return &mavlink_tasks;
}



uint8_t mavlink_receive(byte_stream_t* stream, Mavlink_Received_t* rec) {
	uint8_t byte;
	//dbg_print(".");
	while(buffer_bytes_available(stream->data) > 0) {
		//dbg_print("!");
		byte = stream->get(stream->data);
		if(mavlink_parse_char(MAVLINK_COMM_0, byte, &rec->msg, &rec->status)) {
			return 1;
		}
	}		
	return 0;
}

void handle_mavlink_message(Mavlink_Received_t* rec) {
	switch(rec->msg.msgid) {
		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
			send_all_parameters(rec);
		}
		break;
		case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
			send_parameter(rec);
		}
		break;
		case MAVLINK_MSG_ID_PARAM_SET: {
			receive_parameter(rec);
		}
		break;
		/* 
		TODO : add other cases
		*/
	}
}

