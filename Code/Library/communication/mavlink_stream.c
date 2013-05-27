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
#include "boardsupport.h"
#include "waypoint_navigation.h"

byte_stream_t* mavlink_out_stream;
byte_stream_t* mavlink_in_stream;
Buffer_t mavlink_in_buffer;

board_hardware_t* board;

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
 		dbg_print("\n received message with id");
 		dbg_print_num(rec.msg.msgid, 10);
 		dbg_print(" from system");
 		dbg_print_num(rec.msg.sysid, 10);
		dbg_print(" with comp id");
		dbg_print_num(rec.msg.compid, 10);
 		dbg_print( "\n");
		
		handle_mavlink_message(&rec);
	}
}

void init_mavlink(byte_stream_t *transmit_stream, byte_stream_t *receive_stream) {
	mavlink_system.sysid = 154; // System ID, 1-255
	mavlink_system.compid = 50; // Component/Subsystem ID, 1-255
	mavlink_system.type = MAV_TYPE_QUADROTOR;
	
	mavlink_mission_planner.sysid = mavlink_system.sysid;
	mavlink_mission_planner.compid = MAV_COMP_ID_MISSIONPLANNER;
	mavlink_mission_planner.type = MAV_TYPE_QUADROTOR;
	
	mavlink_out_stream = transmit_stream;
	mavlink_in_stream = receive_stream;
	make_buffered_stream(&mavlink_in_buffer, mavlink_in_stream);
	init_scheduler(&mavlink_tasks);
	
	register_task(&mavlink_tasks, 0, 10000, &mavlink_receive_handler);
	board = get_board_hardware();
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
		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: { // 21
			send_all_parameters(rec);
		}
		break;
		case MAVLINK_MSG_ID_PARAM_REQUEST_READ: { // 20
			send_parameter(rec);
		}
		break;
		case MAVLINK_MSG_ID_PARAM_SET: { // 23
			receive_parameter(rec);
		}
		break;
		case MAVLINK_MSG_ID_MISSION_ITEM: { // 39
			receive_waypoint(rec,&board->waypoint_list,board->number_of_waypoints);
			dbg_print("receive waypoint \n");
		}
		break;
		case MAVLINK_MSG_ID_MISSION_REQUEST : { // 40
			send_waypoint(rec,board->waypoint_list);
			dbg_print("send waypoint\n");
		}
		break;
		case MAVLINK_MSG_ID_MISSION_SET_CURRENT : { // 41
			set_current_wp(rec,&board->waypoint_list);
			dbg_print("set current waypoint\n");
		}
		break;
		case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: { // 43
			send_count(rec,board->number_of_waypoints);
			dbg_print("send count waypoint\n");
		}
		break;
		case MAVLINK_MSG_ID_MISSION_COUNT : { // 44
			receive_count(rec,&board->number_of_waypoints);
			dbg_print("receive count\n");
		}
		break;
		case MAVLINK_MSG_ID_MISSION_CLEAR_ALL : { // 45
			clear_waypoint_list(rec,&board->waypoint_list);
		}
		break;
		case MAVLINK_MSG_ID_MISSION_ACK : { // 47
			receive_ack_msg(rec);
		}
		break;
		/* 
		TODO : add other cases
		*/
	}
}

