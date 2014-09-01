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
 * \file joystick_parsing.c
 *
 * This file is to decode the set manual command message from mavlink
 */


#include "joystick_parsing.h"
#include "print_util.h"


void joystick_parsing_init(joystick_parsing_t* joystick_parsing, control_command_t* controls, mavlink_communication_t* mavlink_communication)
{
	joystick_parsing->controls = controls;
	
	joystick_parsing->controls->rpy[ROLL] = 0.0f;
	joystick_parsing->controls->rpy[PITCH] = 0.0f;
	joystick_parsing->controls->rpy[YAW] = 0.0f;
	joystick_parsing->controls->tvel[X] = 0.0f;
	joystick_parsing->controls->tvel[Y] = 0.0f;
	joystick_parsing->controls->tvel[Z] = 0.0f;
	joystick_parsing->controls->theading = 0.0f;
	joystick_parsing->controls->thrust = -1.0f;
	joystick_parsing->controls->control_mode = ATTITUDE_COMMAND_MODE;
	joystick_parsing->controls->yaw_mode = YAW_ABSOLUTE;
	
	joystick_parsing->buttons = 0;
	
	joystick_parsing->controls->mavlink_stream = &mavlink_communication->mavlink_stream;
	
		// Add callbacks for waypoint handler messages requests
	mavlink_message_handler_msg_callback_t callback;

	callback.message_id 	= MAVLINK_MSG_ID_MANUAL_CONTROL; // 69
	callback.sysid_filter 	= MAVLINK_BASE_STATION_ID;
	callback.compid_filter 	= MAV_COMP_ID_ALL;
	callback.function 		= (mavlink_msg_callback_function_t)	&joystick_parsing_parse_msg;
	callback.module_struct 	= (handling_module_struct_t)		joystick_parsing;
	mavlink_message_handler_add_msg_callback( &mavlink_communication->message_handler, &callback );
	
	
	print_util_dbg_print("Joystick parsing initialised\r");
}

void joystick_parsing_parse_msg(joystick_parsing_t *joystick_parsing, mavlink_received_t* rec)
{
	mavlink_manual_control_t packet;
	mavlink_msg_manual_control_decode(&rec->msg,&packet);
	
	if ((uint8_t)packet.target == (uint8_t)joystick_parsing->controls->mavlink_stream->sysid)
	{
		//print_util_dbg_print("Joystick command: (");
		//print_util_dbg_print_num(packet.x,10);
		//print_util_dbg_print(", ");
		//print_util_dbg_print_num(packet.y,10);
		//print_util_dbg_print(", ");
		//print_util_dbg_print_num(packet.z,10);
		//print_util_dbg_print("), ");
		//print_util_dbg_print_num(packet.buttons,10);
		//print_util_dbg_print(", ");
		//print_util_dbg_print_num(packet.r,10);
		//print_util_dbg_print("\r");
		
		joystick_parsing->controls->rpy[PITCH] = MAX_JOYSTICK_RANGE * packet.x / 1000.0f;
		joystick_parsing->controls->rpy[ROLL] = MAX_JOYSTICK_RANGE * packet.y / 1000.0f;
		joystick_parsing->controls->rpy[YAW] = MAX_JOYSTICK_RANGE * packet.r / 1000.0f;
		joystick_parsing->controls->thrust = MAX_JOYSTICK_RANGE * packet.z / 1000.0f;
	}
}

void joystick_parsing_get_velocity_vector_from_joystick(joystick_parsing_t* joystick_parsing, control_command_t* controls)
{
	controls->tvel[X] = - 10.0f * joystick_parsing->controls->rpy[PITCH];
	controls->tvel[Y] = 10.0f * joystick_parsing->controls->rpy[ROLL];
	controls->tvel[Z] = - 1.5f * joystick_parsing->controls->thrust;
	
	controls->rpy[YAW] = joystick_parsing->controls->rpy[YAW];
}

task_return_t joystick_parsing_send_manual_ctrl_msg(joystick_parsing_t* joystick_parsing)
{
	mavlink_message_t msg;
	mavlink_msg_manual_control_pack(joystick_parsing->controls->mavlink_stream->sysid,
									joystick_parsing->controls->mavlink_stream->compid,
									&msg,
									joystick_parsing->controls->mavlink_stream->sysid,
									joystick_parsing->controls->rpy[PITCH] * 1000,
									joystick_parsing->controls->rpy[ROLL] * 1000,
									joystick_parsing->controls->thrust* 1000, 
									joystick_parsing->controls->rpy[YAW] * 1000,
									joystick_parsing->buttons);
	
	mavlink_stream_send(joystick_parsing->controls->mavlink_stream, &msg);
	
	return TASK_RUN_SUCCESS;
}