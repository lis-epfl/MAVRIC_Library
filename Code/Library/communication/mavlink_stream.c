/*
 * mavlink_stream.c
 *
 * a wrapper for mavlink to use the stream interface
 *
 * Created: 14/02/2013 17:10:05
 *  Author: Felix, Julien, Nicolas
 */ 

#include "mavlink_stream.h"
#include "buffer.h"
#include "onboard_parameters.h"
#include "print_util.h"
#include "central_data.h"
#include "mavlink_actions.h"

static volatile byte_stream_t* mavlink_out_stream;
static volatile byte_stream_t* mavlink_in_stream;
static volatile Buffer_t mavlink_in_buffer;

central_data_t *centralData;

NEW_TASK_SET (mavlink_tasks, 30)

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
		handle_mavlink_message(&rec);
	}
}

void init_mavlink(byte_stream_t *transmit_stream, byte_stream_t *receive_stream, int sysid) {
	mavlink_system.sysid = sysid; // System ID, 1-255
	mavlink_system.compid = 50; // Component/Subsystem ID, 1-255
	mavlink_system.type = MAV_TYPE_QUADROTOR;
	
	mavlink_mission_planner.sysid = mavlink_system.sysid;
	mavlink_mission_planner.compid = MAV_COMP_ID_MISSIONPLANNER;
	mavlink_mission_planner.type = MAV_TYPE_QUADROTOR;
	
	mavlink_out_stream = transmit_stream;
	mavlink_in_stream = receive_stream;
	
	init_scheduler(&mavlink_tasks);
	
	add_task(&mavlink_tasks, 100000, RUN_REGULAR, &send_scheduled_parameters, MAVLINK_MSG_ID_PARAM_VALUE);

	centralData = get_central_data();
}

void flush_mavlink() {
	if (mavlink_out_stream->flush!=NULL) {
		//mavlink_out_stream->buffer_empty(mavlink_out_stream->data);
		mavlink_out_stream->flush(mavlink_out_stream->data);	
	
	}
}

task_return_t mavlink_protocol_update() {
	task_return_t result=0;
	mavlink_receive_handler();
	if ((mavlink_out_stream->buffer_empty(mavlink_out_stream->data))==true) {
		result = run_scheduler_update(&mavlink_tasks, ROUND_ROBIN);
		//flush_mavlink();
	}
		
	
	return result;
}

task_set* get_mavlink_taskset() {
	return &mavlink_tasks;
}

void suspend_downstream(uint32_t delay) {
	int i;
	for (i=0; i<mavlink_tasks.number_of_tasks; i++) {
		suspend_task(&mavlink_tasks.tasks[i], delay);
	}	
}

uint8_t mavlink_receive(byte_stream_t* stream, Mavlink_Received_t* rec) {
	uint8_t byte;
	//dbg_print(" ");
	while(stream->bytes_available(stream->data) > 0) {
		byte = stream->get(stream->data);
		//dbg_print(".");
		//dbg_print_num(byte, 16);
		//dbg_print(" ");
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
	if (rec->msg.sysid == MAVLINK_BASE_STATION_ID) {
		switch(rec->msg.msgid) {
			case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: { // 21
				mavlink_param_request_list_t request;
				mavlink_msg_param_request_list_decode(&rec->msg, &request);
			
				dbg_print("msg comp id:");
				dbg_print_num(request.target_component,10);
				dbg_print("\n");
			
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
				suspend_downstream(100000);
				receive_parameter(rec);
			}
			break;

			case MAVLINK_MSG_ID_REQUEST_DATA_STREAM: { // 66
				mavlink_request_data_stream_t request;
				mavlink_msg_request_data_stream_decode(&rec->msg, &request);
				// TODO: control target_component == compid!
				if ((uint8_t)request.target_system == (uint8_t)mavlink_system.sysid
				&& (uint8_t)request.target_component == (uint8_t)mavlink_system.compid)
				{
					dbg_print("stream request:");
					dbg_print_num(request.target_component,10);
					if (request.req_stream_id==255) {
						int i;
						dbg_print("send all\n");
						// send full list of streams
						for (i=0; i<mavlink_tasks.number_of_tasks; i++) {
							task_entry *task=get_task_by_index(&mavlink_tasks, i);
							run_task_now(task);
						}					
					} else {
						int i;
						task_entry *task=get_task_by_id(&mavlink_tasks, request.req_stream_id);
						dbg_print(" stream="); dbg_print_num(request.req_stream_id, 10);
						dbg_print(" start_stop=");dbg_print_num(request.start_stop, 10);
						dbg_print(" rate=");dbg_print_num(request.req_message_rate,10);
						dbg_print("\n");
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
				}
			}	
			break;
		}
	}

	// handle all platform-specific messages in mavlink-actions:
	handle_specific_messages(rec);
}	
