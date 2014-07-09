/** 
 * \page The MAV'RIC license
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */


/**
 * \file mavlink_message_handler.c
 *
 * This module handles of all incoming mavlink message by calling the appropriate functions
 */

// TODO: update documentation

#include "mavlink_message_handler.h"
#include "print_util.h"

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

void mavlink_message_handler_default_do_nothing(mavlink_received_t* rec);


void mavlink_message_handler_default_print_dbg(mavlink_received_t* rec);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void mavlink_message_handler_default_do_nothing(mavlink_received_t* rec)
{
	;
}


void mavlink_message_handler_default_print_dbg(mavlink_received_t* rec)
{
	print_util_dbg_print("\n Received message with ID");
	print_util_dbg_print_num(rec->msg.msgid, 10);
	print_util_dbg_print(" from system");
	print_util_dbg_print_num(rec->msg.sysid, 10);
	print_util_dbg_print(" for component");
	print_util_dbg_print_num(rec->msg.compid,10);
	print_util_dbg_print( "\n");
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------


void mavlink_message_handler_init(mavlink_message_handler_t* message_handler)
{
	;
}

void mavlink_message_handler_receive(mavlink_message_handler_t* message_handler, mavlink_received_t* rec) 
{
	// if (rec->msg.sysid == MAVLINK_BASE_STATION_ID) 
	// {
	// 	//print_util_dbg_print("\n Received message with ID");
	// 	//print_util_dbg_print_num(rec->msg.msgid, 10);
	// 	//print_util_dbg_print(" from system");
	// 	//print_util_dbg_print_num(rec->msg.sysid, 10);
	// 	//print_util_dbg_print(" for component");
	// 	//print_util_dbg_print_num(rec->msg.compid,10);
	// 	//print_util_dbg_print( "\n");
		
	// 	switch(rec->msg.msgid) 
	// 	{
	// 		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: 
	// 		{ // 21
	// 			mavlink_param_request_list_t request;
	// 			mavlink_msg_param_request_list_decode(&rec->msg, &request);
			
	// 			print_util_dbg_print("msg comp id:");
	// 			print_util_dbg_print_num(request.target_component,10);
	// 			print_util_dbg_print("\n");
			
	// 			// Check if this message is for this system
	// 			if ((uint8_t)request.target_system == (uint8_t)mavlink_system.sysid) 
	// 			{
	// 				print_util_dbg_print("Sending all parameters \n");
	// 				onboard_parameters_send_all_parameters(message_handler->onboard_params);
	// 			}				
	// 		}
	// 		break;
	// 		case MAVLINK_MSG_ID_PARAM_REQUEST_READ: 
	// 		{ //20
	// 			mavlink_param_request_read_t request;
	// 			mavlink_msg_param_request_read_decode(&rec->msg, &request);
	// 			// Check if this message is for this system and subsystem
	// 			if ((uint8_t)request.target_system == (uint8_t)mavlink_system.sysid)
	// 			//&& (uint8_t)request.target_component == (uint8_t)mavlink_system.compid)
	// 			 {
	// 				print_util_dbg_print("Sending parameter ");
	// 				print_util_dbg_print(request.param_id);
	// 				onboard_parameters_send_parameter(message_handler->onboard_params, &request);
	// 			}				
	// 		}
	// 		break;
	// 		case MAVLINK_MSG_ID_PARAM_SET: 
	// 		{ //23
	// 			mavlink_stream_suspend_downstream(100000);
	// 			onboard_parameters_receive_parameter(message_handler->onboard_params, rec);
	// 		}
	// 		break;

	// 		case MAVLINK_MSG_ID_REQUEST_DATA_STREAM: 
	// 		{ // 66
	// 			volatile mavlink_request_data_stream_t request;
	// 			mavlink_msg_request_data_stream_decode(&rec->msg, (mavlink_request_data_stream_t*) &request);
	// 			// TODO: control target_component == compid!
	// 			if ((uint8_t)request.target_system == (uint8_t)mavlink_system.sysid)
	// 			//&& (uint8_t)request.target_component == (uint8_t)mavlink_system.compid)
	// 			{
	// 				print_util_dbg_print("stream request:");
	// 				print_util_dbg_print_num(request.target_component,10);
	// 				if (request.req_stream_id==255) 
	// 				{
	// 					int32_t i;
	// 					print_util_dbg_print("send all\n");
	// 					// send full list of streams
	// 					for (i = 0; i < mavlink_task_set.number_of_tasks; i++) 
	// 					{
	// 						task_entry_t *task=scheduler_get_task_by_index(&mavlink_task_set, i);
	// 						scheduler_run_task_now(task);
	// 					}					
	// 				} 
	// 				else 
	// 				{
	// 					task_entry_t *task =scheduler_get_task_by_id(&mavlink_task_set, request.req_stream_id);
	// 					print_util_dbg_print(" stream="); print_util_dbg_print_num(request.req_stream_id, 10);
	// 					print_util_dbg_print(" start_stop=");print_util_dbg_print_num(request.start_stop, 10);
	// 					print_util_dbg_print(" rate=");print_util_dbg_print_num(request.req_message_rate,10);
	// 					print_util_dbg_print("\n");
	// 					print_util_dbg_print("\n");
	// 					if (request.start_stop) 
	// 					{
	// 						scheduler_change_run_mode(task, RUN_REGULAR);
	// 					}
	// 					else 
	// 					{
	// 						scheduler_change_run_mode(task, RUN_NEVER);
	// 					}
	// 					if (request.req_message_rate>0) 
	// 					{
	// 						scheduler_change_task_period(task, SCHEDULER_TIMEBASE / (uint32_t)request.req_message_rate);
	// 					}
	// 				}
	// 			}
	// 		}	
	// 		break;
	// 	}
	// }

	// // handle all platform-specific messages in mavlink-actions:
	// mavlink_actions_handle_specific_messages(rec);
}