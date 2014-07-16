/**
 * \page The MAV'RIC License
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */


/**
 * \file mavlink_waypoint_handler.c
 * 
 * The mavlink waypoint handler
 */


#include "mavlink_waypoint_handler.h"
#include "print_util.h"
#include "remote_controller.h"
#include "time_keeper.h"
#include "maths.h"

void waypoint_handler_init(mavlink_waypoint_handler_t* waypoint_handler, position_estimator_t* position_estimator, AHRS_t* attitude_estimation, state_structure_t* state_structure, mavlink_communication_t* mavlink_communication)
{
	waypoint_handler->start_timeout = time_keeper_get_millis();
	waypoint_handler->timeout_max_waypoint = 10000;
	
	waypoint_handler->position_estimator = position_estimator;
	waypoint_handler->simulation_mode = &state_structure->simulation_mode;
	waypoint_handler->attitude_estimation = attitude_estimation;
	waypoint_handler->state_structure = state_structure;
	
	waypoint_handler->mavlink_communication = mavlink_communication;
	
	waypoint_handler->critical_behavior = CLIMB_TO_SAFE_ALT;
	waypoint_handler->critical_next_state = false;

	// init waypoint navigation
	// TODO: move to
	// waypoint_handler_init(&waypoint_handler->waypoint_handler);
	// waypoint_handler->waypoint_handler.number_of_waypoints= ...
	waypoint_handler->number_of_waypoints = 0;
	waypoint_handler->num_waypoint_onboard = 0;
	
	waypoint_handler->sending_waypoint_num = 0;
	waypoint_handler->waypoint_request_number = 0;
	
	waypoint_handler->waypoint_set = false;
	waypoint_handler->waypoint_sending = false;
	waypoint_handler->waypoint_receiving = false;
			
	waypoint_handler->critical_landing = false;
			
	waypoint_handler->collision_avoidance = false;
	waypoint_handler->automatic_take_off = false;
	waypoint_handler->automatic_landing = false;
	waypoint_handler->in_the_air = false;
	
	//waypoint_handler_init_waypoint_list(waypoint_handler);
	//waypoint_handler_init_homing_waypoint(waypoint_handler);
	waypoint_handler_waypoint_init(waypoint_handler);
	
	print_util_dbg_print("Waypoint handler init.\n");
	
	// Add callbacks for onboard parameters requests
	mavlink_message_handler_msg_callback_t callback;

	callback.message_id 	= MAVLINK_MSG_ID_SET_MODE; // 11
	callback.sysid_filter 	= MAVLINK_BASE_STATION_ID;
	callback.compid_filter 	= MAV_COMP_ID_ALL;
	callback.function 		= (mavlink_msg_callback_function_t)	&waypoint_handler_set_mav_mode;
	callback.module_struct 	= (handling_module_struct_t)		waypoint_handler;
	mavlink_message_handler_add_msg_callback( &mavlink_communication->message_handler, &callback );

	callback.message_id 	= MAVLINK_MSG_ID_MISSION_ITEM; // 39
	callback.sysid_filter 	= MAVLINK_BASE_STATION_ID;
	callback.compid_filter 	= MAV_COMP_ID_ALL;
	callback.function 		= (mavlink_msg_callback_function_t)	&waypoint_handler_receive_waypoint;
	callback.module_struct 	= (handling_module_struct_t)		waypoint_handler;
	mavlink_message_handler_add_msg_callback( &mavlink_communication->message_handler, &callback );
	
	callback.message_id 	= MAVLINK_MSG_ID_MISSION_REQUEST; // 40
	callback.sysid_filter 	= MAVLINK_BASE_STATION_ID;
	callback.compid_filter 	= MAV_COMP_ID_ALL;
	callback.function 		= (mavlink_msg_callback_function_t)	&waypoint_handler_send_waypoint;
	callback.module_struct 	= (handling_module_struct_t)		waypoint_handler;
	mavlink_message_handler_add_msg_callback( &mavlink_communication->message_handler, &callback );
	
	callback.message_id 	= MAVLINK_MSG_ID_MISSION_SET_CURRENT; // 41
	callback.sysid_filter 	= MAVLINK_BASE_STATION_ID;
	callback.compid_filter 	= MAV_COMP_ID_ALL;
	callback.function 		= (mavlink_msg_callback_function_t)	&waypoint_handler_set_current_waypoint;
	callback.module_struct 	= (handling_module_struct_t)		waypoint_handler;
	mavlink_message_handler_add_msg_callback( &mavlink_communication->message_handler, &callback );
	
	callback.message_id 	= MAVLINK_MSG_ID_MISSION_REQUEST_LIST; // 43
	callback.sysid_filter 	= MAVLINK_BASE_STATION_ID;
	callback.compid_filter 	= MAV_COMP_ID_ALL;
	callback.function 		= (mavlink_msg_callback_function_t)	&waypoint_handler_send_count;
	callback.module_struct 	= (handling_module_struct_t)		waypoint_handler;
	mavlink_message_handler_add_msg_callback( &mavlink_communication->message_handler, &callback );
	
	callback.message_id 	= MAVLINK_MSG_ID_MISSION_COUNT; // 44
	callback.sysid_filter 	= MAVLINK_BASE_STATION_ID;
	callback.compid_filter 	= MAV_COMP_ID_ALL;
	callback.function 		= (mavlink_msg_callback_function_t)	&waypoint_handler_receive_count;
	callback.module_struct 	= (handling_module_struct_t)		waypoint_handler;
	mavlink_message_handler_add_msg_callback( &mavlink_communication->message_handler, &callback );
	
	callback.message_id 	= MAVLINK_MSG_ID_MISSION_CLEAR_ALL; // 45
	callback.sysid_filter 	= MAVLINK_BASE_STATION_ID;
	callback.compid_filter 	= MAV_COMP_ID_ALL;
	callback.function 		= (mavlink_msg_callback_function_t)	&waypoint_handler_clear_waypoint_list;
	callback.module_struct 	= (handling_module_struct_t)		waypoint_handler;
	mavlink_message_handler_add_msg_callback( &mavlink_communication->message_handler, &callback );
	
	callback.message_id 	= MAVLINK_MSG_ID_MISSION_ACK; // 47
	callback.sysid_filter 	= MAVLINK_BASE_STATION_ID;
	callback.compid_filter 	= MAV_COMP_ID_ALL;
	callback.function 		= (mavlink_msg_callback_function_t)	&waypoint_handler_receive_ack_msg;
	callback.module_struct 	= (handling_module_struct_t)		waypoint_handler;
	mavlink_message_handler_add_msg_callback( &mavlink_communication->message_handler, &callback );
	
	callback.message_id 	= MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN; // 48
	callback.sysid_filter 	= MAVLINK_BASE_STATION_ID;
	callback.compid_filter 	= MAV_COMP_ID_ALL;
	callback.function 		= (mavlink_msg_callback_function_t)	&waypoint_handler_set_home;
	callback.module_struct 	= (handling_module_struct_t)		waypoint_handler;
	mavlink_message_handler_add_msg_callback( &mavlink_communication->message_handler, &callback );
}

void waypoint_handler_waypoint_init(mavlink_waypoint_handler_t* waypoint_handler)
{
	uint8_t i,j;
	float rel_pos[3];
	
	if ((waypoint_handler->number_of_waypoints > 0)
	&& (waypoint_handler->position_estimator->init_gps_position || waypoint_handler->simulation_mode)
	&& (waypoint_handler->waypoint_receiving == false))
	{
		for (i=0;i<waypoint_handler->number_of_waypoints;i++)
		{
			if ((waypoint_handler->waypoint_list[i].current == 1)&&(!waypoint_handler->waypoint_set))
			{
				waypoint_handler->current_waypoint_count = i;
				waypoint_handler->current_waypoint = waypoint_handler->waypoint_list[waypoint_handler->current_waypoint_count];
				waypoint_handler->waypoint_coordinates = waypoint_handler_set_waypoint_from_frame(waypoint_handler, waypoint_handler->position_estimator->localPosition.origin);
				
				print_util_dbg_print("Waypoint Nr");
				print_util_dbg_print_num(i,10);
				print_util_dbg_print(" set,\n");
			
				waypoint_handler->waypoint_set = true;
				
				for (j=0;j<3;j++)
				{
					rel_pos[j] = waypoint_handler->waypoint_coordinates.pos[j]-waypoint_handler->position_estimator->localPosition.pos[j];
				}
				waypoint_handler->dist2wp_sqr = vectors_norm_sqr(rel_pos);
			}
		}
	}
}

void waypoint_handler_init_homing_waypoint(mavlink_waypoint_handler_t* waypoint_handler)
{
	waypoint_struct waypoint;
	
	waypoint_handler->number_of_waypoints = 1;
	
	waypoint_handler->num_waypoint_onboard = waypoint_handler->number_of_waypoints;
	
	//Set home waypoint
	waypoint.autocontinue = 0;
	waypoint.current = 1;
	waypoint.frame = MAV_FRAME_LOCAL_NED;
	waypoint.waypoint_id = MAV_CMD_NAV_WAYPOINT;
	
	waypoint.x = 0.0f;
	waypoint.y = 0.0f;
	waypoint.z = -10.0f;
	
	waypoint.param1 = 10; // Hold time in decimal seconds
	waypoint.param2 = 2; // Acceptance radius in meters
	waypoint.param3 = 0; //  0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
	waypoint.param4 = 0; // Desired yaw angle at MISSION (rotary wing)
	
	waypoint_handler->waypoint_list[0] = waypoint;
}

void waypoint_handler_init_waypoint_list(mavlink_waypoint_handler_t* waypoint_handler)
{
	// Visit https://code.google.com/p/ardupilot-mega/wiki/MAVLink to have a description of all messages (or common.h)
	waypoint_struct waypoint;
	
	waypoint_handler->number_of_waypoints = 4;
	
	waypoint_handler->num_waypoint_onboard = waypoint_handler->number_of_waypoints;
	
	// Set nav waypoint
	waypoint.autocontinue = 0;
	waypoint.current = 1;
	waypoint.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
	waypoint.waypoint_id = MAV_CMD_NAV_WAYPOINT;
	
	waypoint.x =  465185223.6174f / 1.0e7f; // convert to deg
	waypoint.y = 65670560 / 1.0e7f; // convert to deg
	waypoint.z = 20; //m
	
	waypoint.param1 = 10; // Hold time in decimal seconds
	waypoint.param2 = 2; // Acceptance radius in meters
	waypoint.param3 = 0; //  0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
	waypoint.param4 = 0; // Desired yaw angle at MISSION (rotary wing)
	
	waypoint_handler->waypoint_list[0] = waypoint;
	
	// Set nav waypoint
	waypoint.autocontinue = 0;
	waypoint.current = 0;
	waypoint.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
	waypoint.waypoint_id = MAV_CMD_NAV_WAYPOINT;
	
	waypoint.x = 465186816 / 1.0e7f; // convert to deg
	waypoint.y = 65670560 / 1.0e7f; // convert to deg
	waypoint.z = 20; //m
	
	waypoint.param1 = 10; // Hold time in decimal seconds
	waypoint.param2 = 4; // Acceptance radius in meters
	waypoint.param3 = 0; //  0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
	waypoint.param4 = 270; // Desired yaw angle at MISSION (rotary wing)
	
	waypoint_handler->waypoint_list[1] = waypoint;
	
	// Set nav waypoint
	waypoint.autocontinue = 1;
	waypoint.current = 0;
	waypoint.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
	waypoint.waypoint_id = MAV_CMD_NAV_WAYPOINT;
	
	waypoint.x = 465186816 / 1.0e7f; // convert to deg
	waypoint.y = 65659084 / 1.0e7f; // convert to deg
	waypoint.z = 180; //m
	
	waypoint.param1 = 10; // Hold time in decimal seconds
	waypoint.param2 = 15; // Acceptance radius in meters
	waypoint.param3 = 0; //  0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
	waypoint.param4 = 90; // Desired yaw angle at MISSION (rotary wing)
	
	waypoint_handler->waypoint_list[2] = waypoint;
	
	// Set nav waypoint
	waypoint.autocontinue = 0;
	waypoint.current = 0;
	waypoint.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
	waypoint.waypoint_id = MAV_CMD_NAV_WAYPOINT;

	waypoint.x = 465182186 / 1.0e7f; // convert to deg
	waypoint.y = 65659084 / 1.0e7f; // convert to deg
	waypoint.z = 20; //m

	waypoint.param1 = 10; // Hold time in decimal seconds
	waypoint.param2 = 12; // Acceptance radius in meters
	waypoint.param3 = 0; //  0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
	waypoint.param4 = 90; // Desired yaw angle at MISSION (rotary wing)

	waypoint_handler->waypoint_list[3] = waypoint;
	
	print_util_dbg_print("Number of Waypoint onboard:");
	print_util_dbg_print_num(waypoint_handler->num_waypoint_onboard,10);
	print_util_dbg_print("\n");
}

void waypoint_handler_send_count(mavlink_waypoint_handler_t* waypoint_handler, mavlink_received_t* rec)
{
	mavlink_communication_suspend_downstream(waypoint_handler->mavlink_communication,500000);
	
	mavlink_mission_request_list_t packet;
	
	mavlink_msg_mission_request_list_decode(&rec->msg,&packet);
	
	// Check if this message is for this system and subsystem
	if (((uint8_t)packet.target_system == (uint8_t)mavlink_mission_planner.sysid)
		&& ((uint8_t)packet.target_component == (uint8_t)mavlink_mission_planner.compid))
	{	
		mavlink_msg_mission_count_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid, waypoint_handler->number_of_waypoints);
		
		if (waypoint_handler->number_of_waypoints != 0)
		{
			waypoint_handler->waypoint_sending = true;
			waypoint_handler->waypoint_receiving = false;
			waypoint_handler->start_timeout = time_keeper_get_millis();
		}
		
		waypoint_handler->sending_waypoint_num = 0;
		print_util_dbg_print("Will send ");
		print_util_dbg_print_num(waypoint_handler->number_of_waypoints,10);
		print_util_dbg_print(" waypoints\n");
	}
}

void waypoint_handler_send_waypoint(mavlink_waypoint_handler_t* waypoint_handler, mavlink_received_t* rec)
{
	if (waypoint_handler->waypoint_sending)
	{
		mavlink_mission_request_t packet;
		
		mavlink_msg_mission_request_decode(&rec->msg,&packet);
		
		print_util_dbg_print("Asking for waypoint number ");
		print_util_dbg_print_num(packet.seq,10);
		print_util_dbg_print("\n");
		
		// Check if this message is for this system and subsystem
		if (((uint8_t)packet.target_system == (uint8_t)mavlink_mission_planner.sysid)
			&& ((uint8_t)packet.target_component == (uint8_t)mavlink_mission_planner.compid))
		{
			waypoint_handler->sending_waypoint_num = packet.seq;
			if (waypoint_handler->sending_waypoint_num < waypoint_handler->number_of_waypoints)
			{
				//	Prototype of the function "mavlink_msg_mission_item_send" found in mavlink_msg_mission_item.h :
				// mavlink_msg_mission_item_send (	mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint16_t seq,
				//									uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1,
				//									float param2, float param3, float param4, float x, float y, float z)
				mavlink_msg_mission_item_send(MAVLINK_COMM_0, rec->msg.sysid, rec->msg.compid, packet.seq,
				waypoint_handler->waypoint_list[waypoint_handler->sending_waypoint_num].frame,	waypoint_handler->waypoint_list[waypoint_handler->sending_waypoint_num].waypoint_id,
				waypoint_handler->waypoint_list[waypoint_handler->sending_waypoint_num].current,	waypoint_handler->waypoint_list[waypoint_handler->sending_waypoint_num].autocontinue,
				waypoint_handler->waypoint_list[waypoint_handler->sending_waypoint_num].param1,	waypoint_handler->waypoint_list[waypoint_handler->sending_waypoint_num].param2,
				waypoint_handler->waypoint_list[waypoint_handler->sending_waypoint_num].param3,	waypoint_handler->waypoint_list[waypoint_handler->sending_waypoint_num].param4,
				waypoint_handler->waypoint_list[waypoint_handler->sending_waypoint_num].x,		waypoint_handler->waypoint_list[waypoint_handler->sending_waypoint_num].y,
				waypoint_handler->waypoint_list[waypoint_handler->sending_waypoint_num].z);
				
				print_util_dbg_print("Sending waypoint ");
				print_util_dbg_print_num(waypoint_handler->sending_waypoint_num, 10);
				print_util_dbg_print("\n");
				
				waypoint_handler->start_timeout = time_keeper_get_millis();
			}			
		}
	}	
}

void waypoint_handler_receive_ack_msg(mavlink_waypoint_handler_t* waypoint_handler, mavlink_received_t* rec)
{
	mavlink_mission_ack_t packet;
	
	mavlink_msg_mission_ack_decode(&rec->msg, &packet);
	
	// Check if this message is for this system and subsystem
	if (((uint8_t)packet.target_system == (uint8_t)mavlink_mission_planner.sysid) 
		&& ((uint8_t)packet.target_component == (uint8_t)mavlink_mission_planner.compid))
	{
		waypoint_handler->waypoint_sending = false;
		waypoint_handler->sending_waypoint_num = 0;
		print_util_dbg_print("Acknowledgment received, end of waypoint sending.\n");
	}
}

void waypoint_handler_receive_count(mavlink_waypoint_handler_t* waypoint_handler, mavlink_received_t* rec)
{
	mavlink_communication_suspend_downstream(waypoint_handler->mavlink_communication,500000);
	
	mavlink_mission_count_t packet;
	
	mavlink_msg_mission_count_decode(&rec->msg, &packet);
	
	print_util_dbg_print("Count:");
	print_util_dbg_print_num(packet.count,10);
	print_util_dbg_print("\n");
	
	// Check if this message is for this system and subsystem
	if (((uint8_t)packet.target_system == (uint8_t)mavlink_mission_planner.sysid)
		&& ((uint8_t)packet.target_component == (uint8_t)mavlink_mission_planner.compid))
	{
		if (waypoint_handler->waypoint_receiving == false)
		{
			// comment these lines if you want to add new waypoints to the list instead of overwriting them
			waypoint_handler->num_waypoint_onboard = 0;
			waypoint_handler->number_of_waypoints =0;
			//---//
			
			if ((packet.count + waypoint_handler->number_of_waypoints) > MAX_WAYPOINTS)
			{
				packet.count = MAX_WAYPOINTS - waypoint_handler->number_of_waypoints;
			}
			waypoint_handler->number_of_waypoints =  packet.count + waypoint_handler->number_of_waypoints;
			print_util_dbg_print("Receiving ");
			print_util_dbg_print_num(packet.count,10);
			print_util_dbg_print(" new waypoints. ");
			print_util_dbg_print("New total number of waypoints:");
			print_util_dbg_print_num(waypoint_handler->number_of_waypoints,10);
			print_util_dbg_print("\n");
			
			waypoint_handler->waypoint_receiving   = true;
			waypoint_handler->waypoint_sending     = false;
			waypoint_handler->waypoint_request_number = 0;
			
			
			waypoint_handler->start_timeout = time_keeper_get_millis();
		}
		
		mavlink_msg_mission_request_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,waypoint_handler->waypoint_request_number);
		
		print_util_dbg_print("Asking for waypoint ");
		print_util_dbg_print_num(waypoint_handler->waypoint_request_number,10);
		print_util_dbg_print("\n");	
	}
	
}

void waypoint_handler_receive_waypoint(mavlink_waypoint_handler_t* waypoint_handler, mavlink_received_t* rec)
{
	mavlink_communication_suspend_downstream(waypoint_handler->mavlink_communication,500000);
	
	mavlink_mission_item_t packet;
	
	mavlink_msg_mission_item_decode(&rec->msg,&packet);
	
	// Check if this message is for this system and subsystem
	if (((uint8_t)packet.target_system == (uint8_t)mavlink_system.sysid)
		&& ((uint8_t)packet.target_component == (uint8_t)mavlink_mission_planner.compid))
	{
		waypoint_handler->start_timeout = time_keeper_get_millis();
		
		waypoint_struct new_waypoint;
		
		new_waypoint.waypoint_id = packet.command;
		
		new_waypoint.x = packet.x; // longitude
		new_waypoint.y = packet.y; // latitude 
		new_waypoint.z = packet.z; // altitude
		
		new_waypoint.autocontinue = packet.autocontinue;
		new_waypoint.frame = packet.frame;
		
		new_waypoint.current = packet.current;
		
		new_waypoint.param1 = packet.param1;
		new_waypoint.param2 = packet.param2;
		new_waypoint.param3 = packet.param3;
		new_waypoint.param4 = packet.param4;
		
		print_util_dbg_print("New waypoint received ");
		//print_util_dbg_print("(");
 		//print_util_dbg_print_num(new_waypoint.x,10);
 		//print_util_dbg_print(", ");
 		//print_util_dbg_print_num(new_waypoint.y,10);
 		//print_util_dbg_print(", ");
 		//print_util_dbg_print_num(new_waypoint.z,10);
 		//print_util_dbg_print(") Autocontinue:");
 		//print_util_dbg_print_num(new_waypoint.autocontinue,10);
 		//print_util_dbg_print(" Frame:");
 		//print_util_dbg_print_num(new_waypoint.frame,10);
 		//print_util_dbg_print(" Current :");
 		//print_util_dbg_print_num(packet.current,10);
 		//print_util_dbg_print(" Seq :");
 		//print_util_dbg_print_num(packet.seq,10);
		//print_util_dbg_print(" command id :");
		//print_util_dbg_print_num(packet.command,10);
		print_util_dbg_print(" requested num :");
		print_util_dbg_print_num(waypoint_handler->waypoint_request_number,10);
		print_util_dbg_print(" receiving num :");
		print_util_dbg_print_num(packet.seq,10);
		//print_util_dbg_print(" is it receiving :");
		//print_util_dbg_print_num(waypoint_handler->waypoint_receiving,10); // boolean value
		print_util_dbg_print("\n");
		
		//current = 2 is a flag to tell us this is a "guided mode" waypoint and not for the mission		
		if(packet.current == 2)
		{                                               
			// verify we received the command
			mavlink_msg_mission_ack_send(MAVLINK_COMM_0, rec->msg.sysid,rec->msg.compid, MAV_CMD_ACK_ERR_NOT_SUPPORTED);
		}
		else
		{
			//current = 3 is a flag to tell us this is a alt change only
			if(packet.current == 3)
			{                                    
				// verify we received the command
				mavlink_msg_mission_ack_send(MAVLINK_COMM_0, rec->msg.sysid,rec->msg.compid, MAV_CMD_ACK_ERR_NOT_SUPPORTED);
			}
			else
			{
			// Check if receiving waypoints
				if (waypoint_handler->waypoint_receiving)
				{

					// check if this is the requested waypoint
					if (packet.seq == waypoint_handler->waypoint_request_number)
					{
						print_util_dbg_print("Receiving good waypoint, number ");
						print_util_dbg_print_num(waypoint_handler->waypoint_request_number,10);
						print_util_dbg_print(" of ");
						print_util_dbg_print_num(waypoint_handler->number_of_waypoints - waypoint_handler->num_waypoint_onboard,10);
						print_util_dbg_print("\n");
					
						waypoint_handler->waypoint_list[waypoint_handler->num_waypoint_onboard + waypoint_handler->waypoint_request_number] = new_waypoint;
						waypoint_handler->waypoint_request_number++;
					
						if ((waypoint_handler->num_waypoint_onboard + waypoint_handler->waypoint_request_number) == waypoint_handler->number_of_waypoints) 
						{
						
								uint8_t type = MAV_CMD_ACK_OK;	//MAV_CMD_ACK_ERR_FAIL;                         
							
							mavlink_msg_mission_ack_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,type);
						
							print_util_dbg_print("flight plan received!\n");
							waypoint_handler->waypoint_receiving = false;
							waypoint_handler->num_waypoint_onboard = waypoint_handler->number_of_waypoints;
							waypoint_handler->waypoint_set = false;
							waypoint_handler_waypoint_init(waypoint_handler);
						}
						else
						{
							mavlink_msg_mission_request_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,waypoint_handler->waypoint_request_number);
						
							print_util_dbg_print("Asking for waypoint ");
							print_util_dbg_print_num(waypoint_handler->waypoint_request_number,10);
							print_util_dbg_print("\n");
						}
					}
				}
				else
				{
					uint8_t type = MAV_CMD_ACK_OK;	//MAV_CMD_ACK_ERR_FAIL;      
					print_util_dbg_print("Ack not received!");
					mavlink_msg_mission_ack_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,type);
			}				
		}		
	}			
}		
}		

void waypoint_handler_set_current_waypoint(mavlink_waypoint_handler_t* waypoint_handler, mavlink_received_t* rec)
{
	mavlink_mission_set_current_t packet;
	
	mavlink_msg_mission_set_current_decode(&rec->msg,&packet);
	
	// Check if this message is for this system and subsystem
	if (((uint8_t)packet.target_system == (uint8_t)mavlink_system.sysid)
		&& ((uint8_t)packet.target_component == (uint8_t)mavlink_mission_planner.compid))
	{
		if (packet.seq < waypoint_handler->number_of_waypoints)
		{
			int32_t i;
			for (i=0;i<waypoint_handler->number_of_waypoints;i++)
			{
				waypoint_handler->waypoint_list[i].current = 0;
			}
			
			waypoint_handler->waypoint_list[packet.seq].current = 1;
			mavlink_msg_mission_current_send(MAVLINK_COMM_0,packet.seq);
			
			print_util_dbg_print("Set current waypoint to number");
			print_util_dbg_print_num(packet.seq,10);
			print_util_dbg_print("\n");
			
			waypoint_handler->waypoint_set = false;
			waypoint_handler_waypoint_init(waypoint_handler);
		}
		else
		{
			mavlink_msg_mission_ack_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,MAV_CMD_ACK_ERR_ACCESS_DENIED);
		}
	}
}

void waypoint_handler_set_current_waypoint_from_parameter(mavlink_waypoint_handler_t* waypoint_handler, uint16_t new_current)
{
	uint8_t i;
	
	if (new_current < waypoint_handler->number_of_waypoints)
	{

		for (i=0;i<waypoint_handler->number_of_waypoints;i++)
		{
			waypoint_handler->waypoint_list[i].current = 0;
		}
		waypoint_handler->waypoint_list[new_current].current = 1;
		mavlink_msg_mission_current_send(MAVLINK_COMM_0,new_current);
		
		print_util_dbg_print("Set current waypoint to number");
		print_util_dbg_print_num(new_current,10);
		print_util_dbg_print("\n");
		
		waypoint_handler->waypoint_set = false;
		waypoint_handler_waypoint_init(waypoint_handler);
	}
	else
	{
		mavlink_msg_mission_ack_send(MAVLINK_COMM_0,mavlink_mission_planner.sysid,mavlink_mission_planner.compid,MAV_CMD_ACK_ERR_NOT_SUPPORTED);
	}
}

void waypoint_handler_clear_waypoint_list(mavlink_waypoint_handler_t* waypoint_handler, mavlink_received_t* rec)
{
	mavlink_mission_clear_all_t packet;
	
	mavlink_msg_mission_clear_all_decode(&rec->msg,&packet);
	
	// Check if this message is for this system and subsystem
	if (((uint8_t)packet.target_system == (uint8_t)mavlink_system.sysid)
		&& ((uint8_t)packet.target_component == (uint8_t)mavlink_mission_planner.compid))
	{
		waypoint_handler->number_of_waypoints = 0;
		waypoint_handler->num_waypoint_onboard = 0;
		waypoint_handler->waypoint_set = 0;
		waypoint_handler_waypoint_hold_init(waypoint_handler, waypoint_handler->position_estimator->localPosition);
		mavlink_msg_mission_ack_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,MAV_CMD_ACK_OK);
		print_util_dbg_print("Cleared Waypoint list.\n");
	}		
}

void waypoint_handler_set_home(mavlink_waypoint_handler_t* waypoint_handler, mavlink_received_t* rec)
{
	mavlink_set_gps_global_origin_t packet;
	
	mavlink_msg_set_gps_global_origin_decode(&rec->msg,&packet);
	
	// Check if this message is for this system and subsystem
	// Due to possible bug from QGroundControl, no check of target_component and compid
	if ((uint8_t)packet.target_system == (uint8_t)mavlink_system.sysid)
	{
		print_util_dbg_print("Set new home location.\n");
		waypoint_handler->position_estimator->localPosition.origin.latitude = (double) packet.latitude / 10000000.0f;
		waypoint_handler->position_estimator->localPosition.origin.longitude = (double) packet.longitude / 10000000.0f;
		waypoint_handler->position_estimator->localPosition.origin.altitude = (float) packet.altitude / 1000.0f;
		
		print_util_dbg_print("New Home location: (");
		print_util_dbg_print_num(waypoint_handler->position_estimator->localPosition.origin.latitude*10000000.0f,10);
		print_util_dbg_print(", ");
		print_util_dbg_print_num(waypoint_handler->position_estimator->localPosition.origin.longitude*10000000.0f,10);
		print_util_dbg_print(", ");
		print_util_dbg_print_num(waypoint_handler->position_estimator->localPosition.origin.altitude*1000.0f,10);
		print_util_dbg_print(")\n");
		
		mavlink_msg_gps_global_origin_send(MAVLINK_COMM_0,
		waypoint_handler->position_estimator->localPosition.origin.latitude*10000000.0f,
		waypoint_handler->position_estimator->localPosition.origin.longitude*10000000.0f,
		waypoint_handler->position_estimator->localPosition.origin.altitude*1000.0f);
	}
}

void waypoint_handler_set_mav_mode(mavlink_waypoint_handler_t* waypoint_handler, mavlink_received_t* rec)
{
	mavlink_set_mode_t packet;
	
	mavlink_msg_set_mode_decode(&rec->msg,&packet);
	
	// Check if this message is for this system and subsystem
	// No component ID in mavlink_set_mode_t so no control
	if ((uint8_t)packet.target_system == (uint8_t)mavlink_system.sysid)
	{
		print_util_dbg_print("base_mode:");
		print_util_dbg_print_num(packet.base_mode,10);
		print_util_dbg_print(", custom mode:");
		print_util_dbg_print_num(packet.custom_mode,10);
		print_util_dbg_print("\n");

		if (waypoint_handler->state_structure->simulation_mode == 0)
		{
			switch(packet.base_mode)
			{
				case MAV_MODE_STABILIZE_DISARMED:
				case MAV_MODE_GUIDED_DISARMED:
				case MAV_MODE_AUTO_DISARMED:
					waypoint_handler->state_structure->mav_state = MAV_STATE_STANDBY;
					waypoint_handler->state_structure->mav_mode = MAV_MODE_MANUAL_DISARMED;
					break;
				
				case MAV_MODE_MANUAL_ARMED:
					if (remote_controller_get_thrust_from_remote()<-0.95f)
					{
						waypoint_handler->state_structure->mav_state = MAV_STATE_ACTIVE;
						waypoint_handler->state_structure->mav_mode = MAV_MODE_MANUAL_ARMED;
					}
					break;
			}
		}
		else
		{
			switch(packet.base_mode)
			{
				case MAV_MODE_STABILIZE_DISARMED:
				case MAV_MODE_GUIDED_DISARMED:
				case MAV_MODE_AUTO_DISARMED:
					waypoint_handler->state_structure->mav_state = MAV_STATE_STANDBY;
					waypoint_handler->state_structure->mav_mode = MAV_MODE_MANUAL_DISARMED;
					break;
				case MAV_MODE_MANUAL_ARMED:
					waypoint_handler->state_structure->mav_state = MAV_STATE_ACTIVE;
					waypoint_handler->state_structure->mav_mode = MAV_MODE_MANUAL_ARMED;
					break;
				case MAV_MODE_STABILIZE_ARMED:
					waypoint_handler->state_structure->mav_state = MAV_STATE_ACTIVE;
					waypoint_handler->state_structure->mav_mode = MAV_MODE_STABILIZE_ARMED;
					break;
				case MAV_MODE_GUIDED_ARMED:
					waypoint_handler->state_structure->mav_state = MAV_STATE_ACTIVE;
					waypoint_handler->state_structure->mav_mode = MAV_MODE_GUIDED_ARMED;
					break;
				case MAV_MODE_AUTO_ARMED:
					waypoint_handler->state_structure->mav_state = MAV_STATE_ACTIVE;
					waypoint_handler->state_structure->mav_mode = MAV_MODE_AUTO_ARMED;
					break;
			}
		}
	}
}

void waypoint_handler_control_time_out_waypoint_msg(mavlink_waypoint_handler_t* waypoint_handler)
{
	if (waypoint_handler->waypoint_sending || waypoint_handler->waypoint_receiving)
	{
		uint32_t tnow = time_keeper_get_millis();
		
		if ((tnow - waypoint_handler->start_timeout) > waypoint_handler->timeout_max_waypoint)
		{
			waypoint_handler->start_timeout = tnow;
			if (waypoint_handler->waypoint_sending)
			{
				waypoint_handler->waypoint_sending = false;
				print_util_dbg_print("Sending waypoint timeout");
			}
			if (waypoint_handler->waypoint_receiving)
			{
				waypoint_handler->waypoint_receiving = false;
				
				print_util_dbg_print("Receiving waypoint timeout");
				waypoint_handler->number_of_waypoints = 0;
				waypoint_handler->num_waypoint_onboard = 0;
			}
		}
	}
}

local_coordinates_t waypoint_handler_set_waypoint_from_frame(mavlink_waypoint_handler_t* waypoint_handler, global_position_t origin)
{
	uint8_t i;
	
	global_position_t waypoint_global;
	local_coordinates_t waypoint_coor;
	
	for (i=0;i<3;i++)
	{
		waypoint_coor.pos[i] = 0.0f;
	}
	waypoint_coor.origin = origin;
	waypoint_coor.heading = deg_to_rad(waypoint_handler->current_waypoint.param4);
	waypoint_coor.timestamp_ms = time_keeper_get_millis();

	switch(waypoint_handler->current_waypoint.frame)
	{
		case MAV_FRAME_GLOBAL:
			waypoint_global.latitude = waypoint_handler->current_waypoint.x;
			waypoint_global.longitude = waypoint_handler->current_waypoint.y;
			waypoint_global.altitude = waypoint_handler->current_waypoint.z;
			waypoint_coor = coord_conventions_global_to_local_position(waypoint_global,origin);
			
			waypoint_coor.heading = deg_to_rad(waypoint_handler->current_waypoint.param4);
			
			print_util_dbg_print("waypoint_global: lat (x1e7):");
			print_util_dbg_print_num(waypoint_global.latitude*10000000,10);
			print_util_dbg_print(" long (x1e7):");
			print_util_dbg_print_num(waypoint_global.longitude*10000000,10);
			print_util_dbg_print(" alt (x1000):");
			print_util_dbg_print_num(waypoint_global.altitude*1000,10);
			print_util_dbg_print(" waypoint_coor: x (x100):");
			print_util_dbg_print_num(waypoint_coor.pos[X]*100,10);
			print_util_dbg_print(", y (x100):");
			print_util_dbg_print_num(waypoint_coor.pos[Y]*100,10);
			print_util_dbg_print(", z (x100):");
			print_util_dbg_print_num(waypoint_coor.pos[Z]*100,10);
			print_util_dbg_print(" localOrigin lat (x1e7):");
			print_util_dbg_print_num(origin.latitude*10000000,10);
			print_util_dbg_print(" long (x1e7):");
			print_util_dbg_print_num(origin.longitude*10000000,10);
			print_util_dbg_print(" alt (x1000):");
			print_util_dbg_print_num(origin.altitude*1000,10);
			print_util_dbg_print("\n");
		
		break;
		case MAV_FRAME_LOCAL_NED:
			waypoint_coor.pos[X] = waypoint_handler->current_waypoint.x;
			waypoint_coor.pos[Y] = waypoint_handler->current_waypoint.y;
			waypoint_coor.pos[Z] = waypoint_handler->current_waypoint.z;
			waypoint_coor.heading= deg_to_rad(waypoint_handler->current_waypoint.param4);
			waypoint_coor.origin = coord_conventions_local_to_global_position(waypoint_coor);
		break;
		case MAV_FRAME_MISSION:
			// Problem here: rec is not defined here
			//mavlink_msg_mission_ack_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,MAV_CMD_ACK_ERR_NOT_SUPPORTED);
		break;
		case MAV_FRAME_GLOBAL_RELATIVE_ALT:
			waypoint_global.latitude = waypoint_handler->current_waypoint.x;
			waypoint_global.longitude = waypoint_handler->current_waypoint.y;
			waypoint_global.altitude = waypoint_handler->current_waypoint.z;
		
			global_position_t origin_relative_alt = origin;
			origin_relative_alt.altitude = 0.0f;
			waypoint_coor = coord_conventions_global_to_local_position(waypoint_global,origin_relative_alt);
			
			waypoint_coor.heading = deg_to_rad(waypoint_handler->current_waypoint.param4);
			
			print_util_dbg_print("LocalOrigin: lat (x1e7):");
			print_util_dbg_print_num(origin_relative_alt.latitude * 10000000,10);
			print_util_dbg_print(" long (x1e7):");
			print_util_dbg_print_num(origin_relative_alt.longitude * 10000000,10);
			print_util_dbg_print(" global alt (x1000):");
			print_util_dbg_print_num(origin.altitude*1000,10);
			print_util_dbg_print(" waypoint_coor: x (x100):");
			print_util_dbg_print_num(waypoint_coor.pos[X]*100,10);
			print_util_dbg_print(", y (x100):");
			print_util_dbg_print_num(waypoint_coor.pos[Y]*100,10);
			print_util_dbg_print(", z (x100):");
			print_util_dbg_print_num(waypoint_coor.pos[Z]*100,10);
			print_util_dbg_print("\n");
		
		break;
		case MAV_FRAME_LOCAL_ENU:
			// Problem here: rec is not defined here
			//mavlink_msg_mission_ack_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,MAV_CMD_ACK_ERR_NOT_SUPPORTED);
		break;
	}
	
	return waypoint_coor;
}

void waypoint_handler_waypoint_hold_init(mavlink_waypoint_handler_t* waypoint_handler, local_coordinates_t localPos)
{
	
	waypoint_handler->waypoint_hold_coordinates = localPos;
	
	//waypoint_handler->waypoint_hold_coordinates.heading = coord_conventions_get_yaw(waypoint_handler->attitude_estimation->qe);
	//waypoint_handler->waypoint_hold_coordinates.heading = localPos.heading;
	
	print_util_dbg_print("Position hold at: (");
	print_util_dbg_print_num(waypoint_handler->waypoint_hold_coordinates.pos[X],10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(waypoint_handler->waypoint_hold_coordinates.pos[Y],10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(waypoint_handler->waypoint_hold_coordinates.pos[Z],10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num((int32_t)(waypoint_handler->waypoint_hold_coordinates.heading*180.0f/3.14f),10);
	print_util_dbg_print(")\n");
	
}

void waypoint_handler_waypoint_take_off(mavlink_waypoint_handler_t* waypoint_handler)
{
	print_util_dbg_print("Automatic take-off. Position hold at: (");
	print_util_dbg_print_num(waypoint_handler->position_estimator->localPosition.pos[X],10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(waypoint_handler->position_estimator->localPosition.pos[Y],10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(-10.0f,10);
	print_util_dbg_print("), with heading of: ");
	print_util_dbg_print_num((int32_t)(waypoint_handler->position_estimator->localPosition.heading*180.0f/3.14f),10);
	print_util_dbg_print("\n");

	waypoint_handler->waypoint_hold_coordinates = waypoint_handler->position_estimator->localPosition;
	waypoint_handler->waypoint_hold_coordinates.pos[Z] = -10.0f;
	
	Aero_Attitude_t aero_attitude;
	aero_attitude=coord_conventions_quat_to_aero(waypoint_handler->attitude_estimation->qe);
	waypoint_handler->waypoint_hold_coordinates.heading = aero_attitude.rpy[2];
	
	waypoint_handler->dist2wp_sqr = 100.0f; // same position, 10m above => distSqr = 100.0f
}

void waypoint_handler_waypoint_hold_position_handler(mavlink_waypoint_handler_t* waypoint_handler)
{
	if (!waypoint_handler->waypoint_set)
	{
		waypoint_handler_waypoint_init(waypoint_handler);
	}
	waypoint_handler_waypoint_hold_init(waypoint_handler, waypoint_handler->position_estimator->localPosition);
}

void waypoint_handler_waypoint_navigation_handler(mavlink_waypoint_handler_t* waypoint_handler)
{

	if (waypoint_handler->waypoint_set)
	{
		uint8_t i;
		float rel_pos[3];
		
		for (i=0;i<3;i++)
		{
			rel_pos[i] = waypoint_handler->waypoint_coordinates.pos[i]-waypoint_handler->position_estimator->localPosition.pos[i];
		}
		waypoint_handler->dist2wp_sqr = vectors_norm_sqr(rel_pos);
		
		if (waypoint_handler->dist2wp_sqr < (waypoint_handler->current_waypoint.param2*waypoint_handler->current_waypoint.param2))
		{
			print_util_dbg_print("Waypoint Nr");
			print_util_dbg_print_num(waypoint_handler->current_waypoint_count,10);
			print_util_dbg_print(" reached, distance:");
			print_util_dbg_print_num(sqrt(waypoint_handler->dist2wp_sqr),10);
			print_util_dbg_print(" less than :");
			print_util_dbg_print_num(waypoint_handler->current_waypoint.param2,10);
			print_util_dbg_print(".\n");
			mavlink_msg_mission_item_reached_send(MAVLINK_COMM_0,waypoint_handler->current_waypoint_count);
			
			waypoint_handler->waypoint_list[waypoint_handler->current_waypoint_count].current = 0;
			if((waypoint_handler->current_waypoint.autocontinue == 1)&&(waypoint_handler->number_of_waypoints>1))
			{
				print_util_dbg_print("Autocontinue towards waypoint Nr");
				
				if (waypoint_handler->current_waypoint_count == (waypoint_handler->number_of_waypoints-1))
				{
					waypoint_handler->current_waypoint_count = 0;
				}
				else
				{
					waypoint_handler->current_waypoint_count++;
				}
				print_util_dbg_print_num(waypoint_handler->current_waypoint_count,10);
				print_util_dbg_print("\n");
				waypoint_handler->waypoint_list[waypoint_handler->current_waypoint_count].current = 1;
				waypoint_handler->current_waypoint = waypoint_handler->waypoint_list[waypoint_handler->current_waypoint_count];
				waypoint_handler->waypoint_coordinates = waypoint_handler_set_waypoint_from_frame(waypoint_handler, waypoint_handler->position_estimator->localPosition.origin);
				
				mavlink_msg_mission_current_send(MAVLINK_COMM_0,waypoint_handler->current_waypoint_count);
				
			}
			else
			{
				waypoint_handler->waypoint_set = false;
				print_util_dbg_print("Stop\n");
				
				waypoint_handler_waypoint_hold_init(waypoint_handler, waypoint_handler->waypoint_coordinates);
			}
		}
	}
	else
	{
		waypoint_handler_waypoint_init(waypoint_handler);
	}
}

void waypoint_handler_waypoint_critical_handler(mavlink_waypoint_handler_t* waypoint_handler)
{
	float rel_pos[3];
	uint8_t i;
	
	if (!(waypoint_handler->critical_next_state))
	{
		waypoint_handler->critical_next_state = true;
		
		Aero_Attitude_t aero_attitude;
		aero_attitude=coord_conventions_quat_to_aero(waypoint_handler->attitude_estimation->qe);
		waypoint_handler->waypoint_critical_coordinates.heading = aero_attitude.rpy[2];
		
		switch (waypoint_handler->critical_behavior)
		{
			case CLIMB_TO_SAFE_ALT:
			waypoint_handler->waypoint_critical_coordinates.pos[X] = waypoint_handler->position_estimator->localPosition.pos[X];
			waypoint_handler->waypoint_critical_coordinates.pos[Y] = waypoint_handler->position_estimator->localPosition.pos[Y];
			waypoint_handler->waypoint_critical_coordinates.pos[Z] = -30.0f;
			
			break;
			case FLY_TO_HOME_WP:
			waypoint_handler->waypoint_critical_coordinates.pos[X] = 0.0f;
			waypoint_handler->waypoint_critical_coordinates.pos[Y] = 0.0f;
			waypoint_handler->waypoint_critical_coordinates.pos[Z] = -30.0f;
			break;
			case CRITICAL_LAND:
			waypoint_handler->waypoint_critical_coordinates.pos[X] = 0.0f;
			waypoint_handler->waypoint_critical_coordinates.pos[Y] = 0.0f;
			waypoint_handler->waypoint_critical_coordinates.pos[Z] = 0.0f;
			break;
		}
		
		for (i=0;i<3;i++)
		{
			rel_pos[i] = waypoint_handler->waypoint_critical_coordinates.pos[i] - waypoint_handler->position_estimator->localPosition.pos[i];
		}
		waypoint_handler->dist2wp_sqr = vectors_norm_sqr(rel_pos);
	}
	
	if (waypoint_handler->dist2wp_sqr < 3.0f)
	{
		waypoint_handler->critical_next_state = false;
		switch (waypoint_handler->critical_behavior)
		{
			case CLIMB_TO_SAFE_ALT:
			print_util_dbg_print("Critical State! Flying to home waypoint.\n");
			waypoint_handler->critical_behavior = FLY_TO_HOME_WP;
			break;
			case FLY_TO_HOME_WP:
			print_util_dbg_print("Critical State! Performing critical landing.\n");
			waypoint_handler->critical_behavior = CRITICAL_LAND;
			break;
			case CRITICAL_LAND:
			print_util_dbg_print("Critical State! Landed, switching off motors, Emergency mode.\n");
			waypoint_handler->critical_landing = true;
			break;
		}
	}
}

void waypoint_handler_auto_landing(mavlink_waypoint_handler_t* waypoint_handler)
{
	float rel_pos[3];
	uint8_t i;
	
	local_coordinates_t local_position;
	
	switch(waypoint_handler->auto_landing_enum)
	{
		case DESCENT_TO_SMALL_ALTITUDE:
			local_position = waypoint_handler->position_estimator->localPosition;
			local_position.pos[Z] = -2.0f;
			
			waypoint_handler_waypoint_hold_init(waypoint_handler, local_position);
			break;
		case DESCENT_TO_GND:
			local_position = waypoint_handler->position_estimator->localPosition;
			local_position.pos[Z] = 0.0f;
			
			waypoint_handler_waypoint_hold_init(waypoint_handler, local_position);
			break;
	}
	
	for (i=0;i<3;i++)
	{
		rel_pos[i] = waypoint_handler->waypoint_critical_coordinates.pos[i] - waypoint_handler->position_estimator->localPosition.pos[i];
	}
	waypoint_handler->dist2wp_sqr = vectors_norm_sqr(rel_pos);
	
	if (waypoint_handler->dist2wp_sqr < 0.5f)
	{
		switch(waypoint_handler->auto_landing_enum)
		{
			case DESCENT_TO_SMALL_ALTITUDE:
				print_util_dbg_print("Automatic-landing: descent_to_GND\n");
				waypoint_handler->critical_behavior = FLY_TO_HOME_WP;
				break;
			case DESCENT_TO_GND:
				
				break;
		}
	}
}

void waypoint_handler_continueToNextWaypoint(mavlink_waypoint_handler_t* waypoint_handler)
{
	if ((waypoint_handler->number_of_waypoints>0)&&(!waypoint_handler->waypoint_set))
	{
		waypoint_handler->waypoint_list[waypoint_handler->current_waypoint_count].current = 0;
		
		print_util_dbg_print("Continuing towards waypoint Nr");
		
		if (waypoint_handler->current_waypoint_count == (waypoint_handler->number_of_waypoints-1))
		{
			waypoint_handler->current_waypoint_count = 0;
		}
		else
		{
			waypoint_handler->current_waypoint_count++;
		}
		print_util_dbg_print_num(waypoint_handler->current_waypoint_count,10);
		print_util_dbg_print("\n");
		waypoint_handler->waypoint_list[waypoint_handler->current_waypoint_count].current = 1;
		waypoint_handler->current_waypoint = waypoint_handler->waypoint_list[waypoint_handler->current_waypoint_count];
		waypoint_handler->waypoint_coordinates = waypoint_handler_set_waypoint_from_frame(waypoint_handler, waypoint_handler->position_estimator->localPosition.origin);
		
		mavlink_msg_mission_current_send(MAVLINK_COMM_0,waypoint_handler->current_waypoint_count);
		
		waypoint_handler->waypoint_set = true;
	}
	else
	{
		print_util_dbg_print("Not ready to switch to next waypoint. Either no waypoint loaded or flying towards one\n");
	}
}

void waypoint_handler_set_circle_scenario(mavlink_waypoint_handler_t* waypoint_handler, float circle_radius, float num_of_vhc)
{
	float angle_step = 2.0 * PI / num_of_vhc;
	
	waypoint_struct waypoint;
	
	local_coordinates_t waypoint_transfo;
	global_position_t waypoint_global;
	
	waypoint_handler->number_of_waypoints = 2;
	waypoint_handler->current_waypoint_count = -1;
	
	waypoint_transfo.origin = waypoint_handler->position_estimator->localPosition.origin;
	
	// Start waypoint
	waypoint_transfo.pos[X] = circle_radius * cos(angle_step * (mavlink_system.sysid-1));
	waypoint_transfo.pos[Y] = circle_radius * sin(angle_step * (mavlink_system.sysid-1));
	waypoint_transfo.pos[Z] = -20.0f;
	waypoint_global = coord_conventions_local_to_global_position(waypoint_transfo);
	
	print_util_dbg_print("Circle departure(x100): (");
	print_util_dbg_print_num(waypoint_transfo.pos[X]*100.0f,10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(waypoint_transfo.pos[Y]*100.0f,10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(waypoint_transfo.pos[Z]*100.0f,10);
	print_util_dbg_print("). For system:");
	print_util_dbg_print_num(mavlink_system.sysid,10);
	print_util_dbg_print(".\n");
	waypoint.x = waypoint_global.latitude;
	waypoint.y = waypoint_global.longitude;
	waypoint.z = waypoint_global.altitude;
	
	waypoint.autocontinue = 0;
	waypoint.current = 0;
	waypoint.frame = MAV_FRAME_GLOBAL;
	waypoint.waypoint_id = MAV_CMD_NAV_WAYPOINT;
	
	waypoint.param1 = 10; // Hold time in decimal seconds
	waypoint.param2 = 4; // Acceptance radius in meters
	waypoint.param3 = 0; //  0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
	waypoint.param4 = rad_to_deg(maths_calc_smaller_angle(PI + angle_step * (mavlink_system.sysid-1))); // Desired yaw angle at MISSION (rotary wing)
	
	waypoint_handler->waypoint_list[0] = waypoint;
	
	// End waypoint
	waypoint_transfo.pos[X] = circle_radius * cos(angle_step * (mavlink_system.sysid-1) + PI);
	waypoint_transfo.pos[Y] = circle_radius * sin(angle_step * (mavlink_system.sysid-1) + PI);
	waypoint_transfo.pos[Z] = -20.0f;
	waypoint_global = coord_conventions_local_to_global_position(waypoint_transfo);
	
	print_util_dbg_print("Circle destination(x100): (");
	print_util_dbg_print_num(waypoint_transfo.pos[X]*100.0f,10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(waypoint_transfo.pos[Y]*100.0f,10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(waypoint_transfo.pos[Z]*100.0f,10);
	print_util_dbg_print("). For system:");
	print_util_dbg_print_num(mavlink_system.sysid,10);
	print_util_dbg_print(".\n");
	
	waypoint.x = waypoint_global.latitude;
	waypoint.y = waypoint_global.longitude;
	waypoint.z = waypoint_global.altitude;
	
	waypoint.autocontinue = 0;
	waypoint.current = 0;
	waypoint.frame = MAV_FRAME_GLOBAL;
	waypoint.waypoint_id = MAV_CMD_NAV_WAYPOINT;
	
	waypoint.param1 = 10; // Hold time in decimal seconds
	waypoint.param2 = 4; // Acceptance radius in meters
	waypoint.param3 = 0; //  0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
	waypoint.param4 = rad_to_deg(angle_step * (mavlink_system.sysid-1)); // Desired yaw angle at MISSION (rotary wing)
	
	waypoint_handler->waypoint_list[1] = waypoint;
	
	waypoint_handler->waypoint_set = false;
}
/*
void set_stream_scenario(waypoint_struct waypoint_list[], uint16_t* number_of_waypoints, float circle_radius, float num_of_vhc)
{
	waypoint_struct waypoint;
	
	// TODO: Add code here :)
}
*/
