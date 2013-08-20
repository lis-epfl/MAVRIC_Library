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

NEW_TASK_SET (mavlink_tasks, 20)

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
		dbg_print("\n Received message with ID");
		dbg_print_num(rec.msg.msgid, 10);
		dbg_print(" from system");
		dbg_print_num(rec.msg.sysid, 10);
		dbg_print(" for component");
		dbg_print_num(rec.msg.compid,10);
		dbg_print( "\n");
		
		handle_mavlink_message(&rec);
		
	}
}

void init_mavlink(byte_stream_t *transmit_stream, byte_stream_t *receive_stream) {
	mavlink_system.sysid = 54; // System ID, 1-255
	mavlink_system.compid = 50; // Component/Subsystem ID, 1-255
	mavlink_system.type = MAV_TYPE_QUADROTOR;
	
	mavlink_mission_planner.sysid = mavlink_system.sysid;
	mavlink_mission_planner.compid = MAV_COMP_ID_MISSIONPLANNER;
	mavlink_mission_planner.type = MAV_TYPE_QUADROTOR;
	
	mavlink_out_stream = transmit_stream;
	mavlink_in_stream = receive_stream;
	make_buffered_stream(&mavlink_in_buffer, mavlink_in_stream);
	init_scheduler(&mavlink_tasks);
	
	add_task(&mavlink_tasks, 500000, RUN_REGULAR, &send_scheduled_parameters, MAVLINK_MSG_ID_PARAM_VALUE);
	board = get_board_hardware();
}

task_return_t mavlink_protocol_update() {
	mavlink_receive_handler();
	if ((mavlink_out_stream->buffer_empty(mavlink_out_stream->data))==true) {
		return run_scheduler_update(&mavlink_tasks, FIXED_PRIORITY);
		if (mavlink_out_stream->flush!=NULL) mavlink_out_stream->flush;
	}	
	return 0;
}

task_set* get_mavlink_taskset() {
	return &mavlink_tasks;
}



uint8_t mavlink_receive(byte_stream_t* stream, Mavlink_Received_t* rec) {
	uint8_t byte;
	//dbg_print(".");
	while(stream->bytes_available(stream->data) > 0) {
		byte = stream->get(stream->data);
		//dbg_print_num(byte, 16);
		//dbg_print("\t");
		if(mavlink_parse_char(MAVLINK_COMM_0, byte, &rec->msg, &rec->status)) {
			//dbg_print("\n");
			return 1;
		}
		//dbg_print_num(rec->status.parse_state, 16);
		//dbg_print("\n");
	}		
	return 0;
}

void handle_mavlink_message(Mavlink_Received_t* rec) {
	
	switch(rec->msg.msgid) {
		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: { // 21
			mavlink_param_request_list_t request;
			mavlink_msg_param_request_list_decode(&rec->msg, &request);
			// Check if this message is for this system
			if ((uint8_t)request.target_system == (uint8_t)mavlink_system.sysid) {
				send_all_parameters();
			}				
		}
		break;
		case MAVLINK_MSG_ID_PARAM_REQUEST_READ: { //20
			mavlink_param_request_read_t request;
			mavlink_msg_param_request_read_decode(&rec->msg, &request);
			// Check if this message is for this system and subsystem
			if ((uint8_t)request.target_system == (uint8_t)mavlink_system.sysid
			&& (uint8_t)request.target_component == (uint8_t)mavlink_system.compid) {

				send_parameter(&request);
			}				
		}
		break;
		case MAVLINK_MSG_ID_PARAM_SET: { //23
			receive_parameter(rec);
		}
		break;
		case MAVLINK_MSG_ID_MISSION_ITEM: { // 39
			receive_waypoint(rec,board->waypoint_list,board->number_of_waypoints);
			//dbg_print("End received waypoint \n");
		}
		break;
		case MAVLINK_MSG_ID_MISSION_REQUEST : { // 40
			send_waypoint(rec,board->waypoint_list);
			dbg_print("Send waypoint\n");
		}
		break;
		case MAVLINK_MSG_ID_MISSION_SET_CURRENT : { // 41
			set_current_wp(rec,&(board->waypoint_list));
			dbg_print("Set current waypoint\n");
		}
		break;
		case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: { // 43
			send_count(rec,board->number_of_waypoints);
			dbg_print("Send count waypoint\n");
		}
		break;
		case MAVLINK_MSG_ID_MISSION_COUNT : { // 44
			receive_count(rec,&(board->number_of_waypoints));
			dbg_print("Receive count, num of waypoints:");
			dbg_print_num(board->number_of_waypoints,10);
			dbg_print("\n");
		}
		break;
		case MAVLINK_MSG_ID_MISSION_CLEAR_ALL : { // 45
			clear_waypoint_list(rec,&board->waypoint_list);
			dbg_print("Clear Waypoint list");
		}
		break;
		case MAVLINK_MSG_ID_MISSION_ACK : { // 47
			receive_ack_msg(rec);
		}
		break;
		case MAVLINK_MSG_ID_SET_MODE : { // 11
			set_mav_mode(rec,&board->mav_mode,&(board->mav_state));
		}
		break;
		case MAVLINK_MSG_ID_COMMAND_LONG : { // 76
			receive_message_long(rec);
		}
		break;			
		case MAVLINK_MSG_ID_REQUEST_DATA_STREAM: { // 66
			mavlink_request_data_stream_t request;
			dbg_print("stream request:");
			mavlink_msg_request_data_stream_decode(&rec->msg, &request);
			dbg_print_num(request.target_component,10);
			if (request.req_stream_id==255) {
				int i;
				dbg_print("send all\n");
				// send full list of streams
				for (i=0; i<mavlink_tasks.number_of_tasks; i++) {
					run_task_now(&mavlink_tasks.tasks[i]);
				}					
			} else {
				task_entry *task=get_task_by_id(&mavlink_tasks, request.req_stream_id);
				dbg_print(" stream="); dbg_print_num(request.req_stream_id, 10);
				dbg_print(" start_stop=");dbg_print_num(request.start_stop, 10);
				dbg_print(" rate=");dbg_print_num(request.req_message_rate,10);
				dbg_print("\n");
				if (request.start_stop) {
					change_run_mode(task, RUN_REGULAR);
				}else {
					change_run_mode(task, RUN_NEVER);
				}
				if (request.req_message_rate>0) {
					change_task_period(task, SCHEDULER_TIMEBASE/(uint32_t)request.req_message_rate);
				}
			}			
			break;
		}
		/* 
		TODO : add other cases
		*/
	}
}

