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
#include "central_data.h"
#include "maths.h"

central_data_t *centralData;

void init_waypoint_handler()
{
	start_timeout = get_millis();
	timeout_max_waypoint = 10000;
	centralData = central_data_get_pointer_to_struct();
	
	centralData->critical_behavior = CLIMB_TO_SAFE_ALT;
	centralData->critical_next_state = false;
	
	//init_waypoint_list(centralData->waypoint_list, &centralData->number_of_waypoints);
	init_homing_waypoint(centralData->waypoint_list, &centralData->number_of_waypoints);
	
	dbg_print("Nav init\n");
	init_waypoint();
}

void init_waypoint()
{
	uint8_t i,j;
	float rel_pos[3];
	
	if (((centralData->number_of_waypoints > 0) 
		&& (centralData->position_estimator.init_gps_position || centralData->simulation_mode) 
		&& centralData->waypoint_receiving) == false)
	{
		for (i=0;i<centralData->number_of_waypoints;i++)
		{
			if ((centralData->waypoint_list[i].current == 1) && (!centralData->waypoint_set))
			{
				centralData->current_waypoint_count = i;
				centralData->current_waypoint = centralData->waypoint_list[centralData->current_waypoint_count];
				centralData->waypoint_coordinates = set_waypoint_from_frame(centralData->current_waypoint,centralData->position_estimator.localPosition.origin);
				
				dbg_print("Waypoint Nr");
				dbg_print_num(i,10);
				dbg_print(" set,\n");
			
				centralData->waypoint_set = true;
				
				for (j=0;j<3;j++)
				{
					rel_pos[j] = centralData->waypoint_coordinates.pos[j]-centralData->position_estimator.localPosition.pos[j];
				}
				centralData->dist2wp_sqr = vector_norm_sqr(rel_pos);
			}
		}
	}
}

void init_homing_waypoint(waypoint_struct waypoint_list[], uint16_t* number_of_waypoints)
{
	waypoint_struct waypoint;
	
	*number_of_waypoints = 1;
	
	num_waypoint_onboard = *number_of_waypoints;
	
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
	
	waypoint_list[0] = waypoint;
}

void init_waypoint_list(waypoint_struct waypoint_list[], uint16_t* number_of_waypoints)
{
	// Visit https://code.google.com/p/ardupilot-mega/wiki/MAVLink to have a description of all messages (or common.h)
	waypoint_struct waypoint;
	
	*number_of_waypoints = 4;
	
	num_waypoint_onboard = *number_of_waypoints;
	
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
	
	waypoint_list[0] = waypoint;
	
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
	
	waypoint_list[1] = waypoint;
	
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
	
	waypoint_list[2] = waypoint;
	
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

	waypoint_list[3] = waypoint;
	
	dbg_print("Number of Waypoint onboard:");
	dbg_print_num(num_waypoint_onboard,10);
	dbg_print("\n");
}

void send_count(Mavlink_Received_t* rec, uint16_t num_of_waypoint, bool* waypoint_receiving, bool * waypoint_sending)
{
	mavlink_mission_request_list_t packet;
	
	mavlink_msg_mission_request_list_decode(&rec->msg,&packet);
	
	// Check if this message is for this system and subsystem
	if (((uint8_t)packet.target_system == (uint8_t)mavlink_mission_planner.sysid)
		&& ((uint8_t)packet.target_component == (uint8_t)mavlink_mission_planner.compid))
	{	
		mavlink_msg_mission_count_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,num_of_waypoint);
		
		if (num_of_waypoint != 0)
		{
			*waypoint_sending = true;
			*waypoint_receiving = false;
			start_timeout = get_millis();
		}
		
		sending_waypoint_num = 0;
		dbg_print("Will send ");
		dbg_print_num(num_of_waypoint,10);
		dbg_print(" waypoints\n");
	}
}

void send_waypoint(Mavlink_Received_t* rec, waypoint_struct waypoint[], uint16_t num_of_waypoint, bool* waypoint_sending)
{
	if (*waypoint_sending)
	{
		mavlink_mission_request_t packet;
		
		mavlink_msg_mission_request_decode(&rec->msg,&packet);
		
		dbg_print("Asking for waypoint number ");
		dbg_print_num(packet.seq,10);
		dbg_print("\n");
		
		// Check if this message is for this system and subsystem
		if (((uint8_t)packet.target_system == (uint8_t)mavlink_mission_planner.sysid)
			&& ((uint8_t)packet.target_component == (uint8_t)mavlink_mission_planner.compid))
		{
			sending_waypoint_num = packet.seq;
			if (sending_waypoint_num < num_of_waypoint)
			{
				//	Prototype of the function "mavlink_msg_mission_item_send" found in mavlink_msg_mission_item.h :
				//		mavlink_msg_mission_item_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z)
				mavlink_msg_mission_item_send(MAVLINK_COMM_0, rec->msg.sysid, rec->msg.compid, packet.seq,
				waypoint[sending_waypoint_num].frame,   waypoint[sending_waypoint_num].waypoint_id,
				waypoint[sending_waypoint_num].current, waypoint[sending_waypoint_num].autocontinue,
				waypoint[sending_waypoint_num].param1,  waypoint[sending_waypoint_num].param2,       waypoint[sending_waypoint_num].param3,    waypoint[sending_waypoint_num].param4,
				waypoint[sending_waypoint_num].x,       waypoint[sending_waypoint_num].y,            waypoint[sending_waypoint_num].z);
				
				dbg_print("Sending waypoint ");
				dbg_print_num(sending_waypoint_num, 10);
				dbg_print("\n");
				
				start_timeout = get_millis();
			}			
		}
	}	
}

void receive_ack_msg(Mavlink_Received_t* rec, bool* waypoint_sending)
{
	mavlink_mission_ack_t packet;
	
	mavlink_msg_mission_ack_decode(&rec->msg, &packet);
	
	// Check if this message is for this system and subsystem
	if (((uint8_t)packet.target_system == (uint8_t)mavlink_mission_planner.sysid) 
		&& ((uint8_t)packet.target_component == (uint8_t)mavlink_mission_planner.compid))
	{
		*waypoint_sending = false;
		sending_waypoint_num = 0;
		dbg_print("Acknowledgment received, end of waypoint sending.\n");
	}
}

void receive_count(Mavlink_Received_t* rec, uint16_t* number_of_waypoints, bool* waypoint_receiving, bool* waypoint_sending)
{
	mavlink_mission_count_t packet;
	
	mavlink_msg_mission_count_decode(&rec->msg, &packet);
	
	// Check if this message is for this system and subsystem
	if (((uint8_t)packet.target_system == (uint8_t)mavlink_mission_planner.sysid)
		&& ((uint8_t)packet.target_component == (uint8_t)mavlink_mission_planner.compid))
	{
		if (*waypoint_receiving == false)
		{
			// comment these lines if you want to add new waypoints to the list instead of overwriting them
			num_waypoint_onboard = 0;
			*number_of_waypoints =0;
			//---//
			
			if ((packet.count + *number_of_waypoints) > MAX_WAYPOINTS)
			{
				packet.count = MAX_WAYPOINTS - *number_of_waypoints;
			}
			*number_of_waypoints =  packet.count+ *number_of_waypoints;
			dbg_print("Receiving ");
			dbg_print_num(packet.count,10);
			dbg_print(" new waypoints. ");
			dbg_print("New total number of waypoints:");
			dbg_print_num(*number_of_waypoints,10);
			dbg_print("\n");
			
			*waypoint_receiving   = true;
			*waypoint_sending     = false;
			waypoint_request_number = 0;
			
			
			start_timeout = get_millis();
		}
		
		mavlink_msg_mission_request_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,waypoint_request_number);
		
		dbg_print("Asking for waypoint ");
		dbg_print_num(waypoint_request_number,10);
		dbg_print("\n");	
	}
	
}

void receive_waypoint(Mavlink_Received_t* rec,  waypoint_struct waypoint_list[], uint16_t number_of_waypoints, bool* waypoint_receiving)
{
	mavlink_mission_item_t packet;
	
	mavlink_msg_mission_item_decode(&rec->msg,&packet);
	
	// Check if this message is for this system and subsystem
	if (((uint8_t)packet.target_system == (uint8_t)mavlink_system.sysid)
		&& ((uint8_t)packet.target_component == (uint8_t)mavlink_mission_planner.compid))
	{
		start_timeout = get_millis();
		
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
		
		dbg_print("New waypoint received ");
		dbg_print(" requested num :");
		dbg_print_num(waypoint_request_number,10);
		dbg_print(" receiving num :");
		dbg_print_num(packet.seq,10);
		dbg_print("\n");
		
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
				if (*waypoint_receiving)
				{

					// check if this is the requested waypoint
					if (packet.seq == waypoint_request_number)
					{
						dbg_print("Receiving good waypoint, number ");
						dbg_print_num(waypoint_request_number,10);
						dbg_print(" of ");
						dbg_print_num(number_of_waypoints-num_waypoint_onboard,10);
						dbg_print("\n");
						
						waypoint_list[num_waypoint_onboard + waypoint_request_number] = new_waypoint;
						waypoint_request_number++;
						
						if ((num_waypoint_onboard + waypoint_request_number) == number_of_waypoints) 
						{
						
							uint8_t type = MAV_CMD_ACK_OK;	//MAV_CMD_ACK_ERR_FAIL;                         
							
							mavlink_msg_mission_ack_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,type);
							
							dbg_print("flight plan received!\n");
							*waypoint_receiving = false;
							num_waypoint_onboard = number_of_waypoints;
							centralData->waypoint_set = false;
							init_waypoint();
						}
						else
						{
							mavlink_msg_mission_request_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,waypoint_request_number);
							
							dbg_print("Asking for waypoint ");
							dbg_print_num(waypoint_request_number,10);
							dbg_print("\n");
						}
					}
				}
				else
				{
					uint8_t type = MAV_CMD_ACK_OK;	//MAV_CMD_ACK_ERR_FAIL;      
					dbg_print("Ack not received!");
					mavlink_msg_mission_ack_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,type);
				}
			}
		}		
	}			
}		

void set_current_waypoint(Mavlink_Received_t* rec,  waypoint_struct waypoint_list[], uint16_t num_of_waypoint)
{
	mavlink_mission_set_current_t packet;
	
	mavlink_msg_mission_set_current_decode(&rec->msg,&packet);
	
	// Check if this message is for this system and subsystem
	if (((uint8_t)packet.target_system == (uint8_t)mavlink_system.sysid)
		&& ((uint8_t)packet.target_component == (uint8_t)mavlink_mission_planner.compid))
	{
		if (packet.seq < num_of_waypoint)
		{
			int i;
			for (i=0;i<num_of_waypoint;i++)
			{
				waypoint_list[i].current = 0;
			}
			
			waypoint_list[packet.seq].current = 1;
			mavlink_msg_mission_current_send(MAVLINK_COMM_0,packet.seq);
			
			dbg_print("Set current waypoint to number");
			dbg_print_num(packet.seq,10);
			dbg_print("\n");
			
			centralData->waypoint_set = false;
			init_waypoint();
		}
		else
		{
			mavlink_msg_mission_ack_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,MAV_CMD_ACK_ERR_ACCESS_DENIED);
		}
	}
}

void set_current_waypoint_from_parameter(waypoint_struct waypoint_list[], uint16_t num_of_waypoint, uint16_t new_current)
{
	uint8_t i;
	
	if (new_current < num_of_waypoint)
	{

		for (i=0;i<num_of_waypoint;i++)
		{
			waypoint_list[i].current = 0;
		}
		waypoint_list[new_current].current = 1;
		mavlink_msg_mission_current_send(MAVLINK_COMM_0,new_current);
		
		dbg_print("Set current waypoint to number");
		dbg_print_num(new_current,10);
		dbg_print("\n");
		
		centralData->waypoint_set = false;
		init_waypoint();
	}
	else
	{
		mavlink_msg_mission_ack_send(MAVLINK_COMM_0,mavlink_mission_planner.sysid,mavlink_mission_planner.compid,MAV_CMD_ACK_ERR_NOT_SUPPORTED);
	}
}

void clear_waypoint_list(Mavlink_Received_t* rec,  uint16_t* number_of_waypoints, bool* waypoint_set)
{
	mavlink_mission_clear_all_t packet;
	
	mavlink_msg_mission_clear_all_decode(&rec->msg,&packet);
	
	// Check if this message is for this system and subsystem
	if (((uint8_t)packet.target_system == (uint8_t)mavlink_system.sysid)
		&& ((uint8_t)packet.target_component == (uint8_t)mavlink_mission_planner.compid))
	{
		*number_of_waypoints = 0;
		num_waypoint_onboard = 0;
		*waypoint_set = 0;
		waypoint_hold_init(centralData->position_estimator.localPosition);
		mavlink_msg_mission_ack_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,MAV_CMD_ACK_OK);
		dbg_print("Cleared Waypoint list.\n");
	}		
}

void set_home(Mavlink_Received_t* rec)
{
	mavlink_set_gps_global_origin_t packet;
	
	mavlink_msg_set_gps_global_origin_decode(&rec->msg,&packet);
	
	// Check if this message is for this system and subsystem
	// Due to possible bug from QGroundControl, no check of target_component and compid
	if ((uint8_t)packet.target_system == (uint8_t)mavlink_system.sysid)
	{
		dbg_print("Set new home location.\n");
		centralData->position_estimator.localPosition.origin.latitude = (double) packet.latitude / 10000000.0f;
		centralData->position_estimator.localPosition.origin.longitude = (double) packet.longitude / 10000000.0f;
		centralData->position_estimator.localPosition.origin.altitude = (float) packet.altitude / 1000.0f;
		
		dbg_print("New Home location: (");
		dbg_print_num(centralData->position_estimator.localPosition.origin.latitude*10000000.0f,10);
		dbg_print(", ");
		dbg_print_num(centralData->position_estimator.localPosition.origin.longitude*10000000.0f,10);
		dbg_print(", ");
		dbg_print_num(centralData->position_estimator.localPosition.origin.altitude*1000.0f,10);
		dbg_print(")\n");
		
		mavlink_msg_gps_global_origin_send(MAVLINK_COMM_0,
		centralData->position_estimator.localPosition.origin.latitude*10000000.0f,
		centralData->position_estimator.localPosition.origin.longitude*10000000.0f,
		centralData->position_estimator.localPosition.origin.altitude*1000.0f);
	}
}

void set_mav_mode(Mavlink_Received_t* rec, uint8_t* board_mav_mode, uint8_t* board_mav_state, uint8_t sim_mode)
{
	mavlink_set_mode_t packet;
	
	mavlink_msg_set_mode_decode(&rec->msg,&packet);
	
	// Check if this message is for this system and subsystem
	// Due to possible bug from QGroundControl, no check of target_component and compid
	if ((uint8_t)packet.target_system == (uint8_t)mavlink_system.sysid)
	{
		dbg_print("base_mode:");
		dbg_print_num(packet.base_mode,10);
		dbg_print(", custom mode:");
		dbg_print_num(packet.custom_mode,10);
		dbg_print("\n");

		if (sim_mode == 0)
		{
			switch(packet.base_mode)
			{
				case MAV_MODE_STABILIZE_DISARMED:
				case MAV_MODE_GUIDED_DISARMED:
				case MAV_MODE_AUTO_DISARMED:
					*board_mav_state = MAV_STATE_STANDBY;
					*board_mav_mode = MAV_MODE_MANUAL_DISARMED;
				break;
				
				case MAV_MODE_MANUAL_ARMED:
					if (get_thrust_from_remote()<-0.95f)
					{
						*board_mav_state = MAV_STATE_ACTIVE;
						*board_mav_mode = MAV_MODE_MANUAL_ARMED;
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
					*board_mav_state = MAV_STATE_STANDBY;
					*board_mav_mode = MAV_MODE_MANUAL_DISARMED;
				break;
				case MAV_MODE_MANUAL_ARMED:
					*board_mav_state = MAV_STATE_ACTIVE;
					*board_mav_mode = MAV_MODE_MANUAL_ARMED;
				break;
				case MAV_MODE_STABILIZE_ARMED:
					*board_mav_state = MAV_STATE_ACTIVE;
					*board_mav_mode = MAV_MODE_STABILIZE_ARMED;
				break;
				case MAV_MODE_GUIDED_ARMED:
					*board_mav_state = MAV_STATE_ACTIVE;
					*board_mav_mode = MAV_MODE_GUIDED_ARMED;
				break;
				case MAV_MODE_AUTO_ARMED:
					*board_mav_state = MAV_STATE_ACTIVE;
					*board_mav_mode = MAV_MODE_AUTO_ARMED;
				break;
			}
		}
	}
}

void control_time_out_waypoint_msg(uint16_t* num_of_waypoint, bool* waypoint_receiving, bool* waypoint_sending)
{
	if (*waypoint_sending || *waypoint_receiving)
	{
		uint32_t tnow = get_millis();
		
		if ((tnow - start_timeout) > timeout_max_waypoint)
		{
			start_timeout = tnow;
			if (*waypoint_sending)
			{
				*waypoint_sending = false;
				dbg_print("Sending waypoint timeout");
			}
			if (*waypoint_receiving)
			{
				*waypoint_receiving = false;
				
				dbg_print("Receiving waypoint timeout");
				*num_of_waypoint = 0;
				num_waypoint_onboard = 0;
			}
		}
	}
}

local_coordinates_t set_waypoint_from_frame(waypoint_struct current_waypoint, global_position_t origin)
{
	uint8_t i;
	
	global_position_t waypoint_global;
	local_coordinates_t waypoint_coor;
	
	float rel_pos[3];
	
	for (i=0;i<3;i++)
	{
		waypoint_coor.pos[i] = 0.0f;
	}

	switch(current_waypoint.frame)
	{
		case MAV_FRAME_GLOBAL:
			waypoint_global.latitude = current_waypoint.x;
			waypoint_global.longitude = current_waypoint.y;
			waypoint_global.altitude = current_waypoint.z;
			waypoint_coor = global_to_local_position(waypoint_global,origin);
			
			waypoint_coor.heading = deg_to_rad(current_waypoint.param4);
			
			dbg_print("waypoint_global: lat (x1e7):");
			dbg_print_num(waypoint_global.latitude*10000000,10);
			dbg_print(" long (x1e7):");
			dbg_print_num(waypoint_global.longitude*10000000,10);
			dbg_print(" alt (x1000):");
			dbg_print_num(waypoint_global.altitude*1000,10);
			dbg_print(" waypoint_coor: x (x100):");
			dbg_print_num(waypoint_coor.pos[X]*100,10);
			dbg_print(", y (x100):");
			dbg_print_num(waypoint_coor.pos[Y]*100,10);
			dbg_print(", z (x100):");
			dbg_print_num(waypoint_coor.pos[Z]*100,10);
			dbg_print(" localOrigin lat (x1e7):");
			dbg_print_num(origin.latitude*10000000,10);
			dbg_print(" long (x1e7):");
			dbg_print_num(origin.longitude*10000000,10);
			dbg_print(" alt (x1000):");
			dbg_print_num(origin.altitude*1000,10);
			dbg_print("\n");
		
		break;
		case MAV_FRAME_LOCAL_NED:
			waypoint_coor.pos[X] = current_waypoint.x;
			waypoint_coor.pos[Y] = current_waypoint.y;
			waypoint_coor.pos[Z] = current_waypoint.z;
			waypoint_coor.heading= deg_to_rad(current_waypoint.param4);
			waypoint_coor.origin = local_to_global_position(waypoint_coor);
		break;
		case MAV_FRAME_MISSION:
			// Problem here: rec is not defined here
			//mavlink_msg_mission_ack_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,MAV_CMD_ACK_ERR_NOT_SUPPORTED);
		break;
		case MAV_FRAME_GLOBAL_RELATIVE_ALT:
			waypoint_global.latitude = current_waypoint.x;
			waypoint_global.longitude = current_waypoint.y;
			waypoint_global.altitude = current_waypoint.z;
		
			global_position_t origin_relative_alt = origin;
			origin_relative_alt.altitude = 0.0f;
			waypoint_coor = global_to_local_position(waypoint_global,origin_relative_alt);
			
			waypoint_coor.heading = deg_to_rad(current_waypoint.param4);
			
			dbg_print("LocalOrigin: lat (x1e7):");
			dbg_print_num(origin_relative_alt.latitude * 10000000,10);
			dbg_print(" long (x1e7):");
			dbg_print_num(origin_relative_alt.longitude * 10000000,10);
			dbg_print(" global alt (x1000):");
			dbg_print_num(origin.altitude*1000,10);
			dbg_print(" waypoint_coor: x (x100):");
			dbg_print_num(waypoint_coor.pos[X]*100,10);
			dbg_print(", y (x100):");
			dbg_print_num(waypoint_coor.pos[Y]*100,10);
			dbg_print(", z (x100):");
			dbg_print_num(waypoint_coor.pos[Z]*100,10);
			dbg_print("\n");
		
		break;
		case MAV_FRAME_LOCAL_ENU:
			// Problem here: rec is not defined here
			//mavlink_msg_mission_ack_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,MAV_CMD_ACK_ERR_NOT_SUPPORTED);
		break;
	}
	
	return waypoint_coor;
}

void waypoint_hold_init(local_coordinates_t localPos)
{
	
	centralData->waypoint_hold_coordinates = localPos;
	
	dbg_print("Position hold at: (");
	dbg_print_num(centralData->waypoint_hold_coordinates.pos[X],10);
	dbg_print(", ");
	dbg_print_num(centralData->waypoint_hold_coordinates.pos[Y],10);
	dbg_print(", ");
	dbg_print_num(centralData->waypoint_hold_coordinates.pos[Z],10);
	dbg_print(", ");
	dbg_print_num((int)(centralData->waypoint_hold_coordinates.heading*180.0f/3.14f),10);
	dbg_print(")\n");
	
}

void waypoint_take_off()
{
	dbg_print("Automatic take-off. Position hold at: (");
	dbg_print_num(centralData->position_estimator.localPosition.pos[X],10);
	dbg_print(", ");
	dbg_print_num(centralData->position_estimator.localPosition.pos[Y],10);
	dbg_print(", ");
	dbg_print_num(-10.0f,10);
	dbg_print("), with heading of: ");
	dbg_print_num((int)(centralData->position_estimator.localPosition.heading*180.0f/3.14f),10);
	dbg_print("\n");

	centralData->waypoint_hold_coordinates = centralData->position_estimator.localPosition;
	centralData->waypoint_hold_coordinates.pos[Z] = -10.0f;
	
	Aero_Attitude_t aero_attitude;
	aero_attitude=Quat_to_Aero(centralData->imu1.attitude.qe);
	centralData->waypoint_hold_coordinates.heading = aero_attitude.rpy[2];
	
	centralData->dist2wp_sqr = 100.0f; // same position, 10m above => distSqr = 100.0f
}


void waypoint_hold_position_handler()
{
	if (!centralData->waypoint_set)
	{
		init_waypoint();
	}
	waypoint_hold_init(centralData->position_estimator.localPosition);
}

void waypoint_navigation_handler()
{

	if (centralData->waypoint_set)
	{
		uint8_t i;
		float rel_pos[3];
		
		for (i=0;i<3;i++)
		{
			rel_pos[i] = centralData->waypoint_coordinates.pos[i]-centralData->position_estimator.localPosition.pos[i];
		}
		centralData->dist2wp_sqr = vector_norm_sqr(rel_pos);
		
		if (centralData->dist2wp_sqr < (centralData->current_waypoint.param2*centralData->current_waypoint.param2))
		{
			dbg_print("Waypoint Nr");
			dbg_print_num(centralData->current_waypoint_count,10);
			dbg_print(" reached, distance:");
			dbg_print_num(sqrt(centralData->dist2wp_sqr),10);
			dbg_print(" less than :");
			dbg_print_num(centralData->current_waypoint.param2,10);
			dbg_print(".\n");
			mavlink_msg_mission_item_reached_send(MAVLINK_COMM_0,centralData->current_waypoint_count);
			
			centralData->waypoint_list[centralData->current_waypoint_count].current = 0;
			if((centralData->current_waypoint.autocontinue == 1)&&(centralData->number_of_waypoints>1))
			{
				dbg_print("Autocontinue towards waypoint Nr");
				
				if (centralData->current_waypoint_count == (centralData->number_of_waypoints-1))
				{
					centralData->current_waypoint_count = 0;
				}
				else
				{
					centralData->current_waypoint_count++;
				}
				dbg_print_num(centralData->current_waypoint_count,10);
				dbg_print("\n");
				centralData->waypoint_list[centralData->current_waypoint_count].current = 1;
				centralData->current_waypoint = centralData->waypoint_list[centralData->current_waypoint_count];
				centralData->waypoint_coordinates = set_waypoint_from_frame(centralData->current_waypoint,centralData->position_estimator.localPosition.origin);
				
				mavlink_msg_mission_current_send(MAVLINK_COMM_0,centralData->current_waypoint_count);
				
			}
			else
			{
				centralData->waypoint_set = false;
				dbg_print("Stop\n");
				
				waypoint_hold_init(centralData->waypoint_coordinates);
			}
		}
	}
}

void waypoint_critical_handler()
{
	float rel_pos[3];
	uint8_t i;
	
	if (!(centralData->critical_next_state))
	{
		centralData->critical_next_state = true;
		
		Aero_Attitude_t aero_attitude;
		aero_attitude=Quat_to_Aero(centralData->imu1.attitude.qe);
		centralData->waypoint_critical_coordinates.heading = aero_attitude.rpy[2];
		
		switch (centralData->critical_behavior)
		{
			case CLIMB_TO_SAFE_ALT:
			centralData->waypoint_critical_coordinates.pos[X] = centralData->position_estimator.localPosition.pos[X];
			centralData->waypoint_critical_coordinates.pos[Y] = centralData->position_estimator.localPosition.pos[Y];
			centralData->waypoint_critical_coordinates.pos[Z] = -30.0f;
			
			break;
			case FLY_TO_HOME_WP:
			centralData->waypoint_critical_coordinates.pos[X] = 0.0f;
			centralData->waypoint_critical_coordinates.pos[Y] = 0.0f;
			centralData->waypoint_critical_coordinates.pos[Z] = -30.0f;
			break;
			case CRITICAL_LAND:
			centralData->waypoint_critical_coordinates.pos[X] = 0.0f;
			centralData->waypoint_critical_coordinates.pos[Y] = 0.0f;
			centralData->waypoint_critical_coordinates.pos[Z] = 0.0f;
			break;
		}
		
		for (i=0;i<3;i++)
		{
			rel_pos[i] = centralData->waypoint_critical_coordinates.pos[i] - centralData->position_estimator.localPosition.pos[i];
		}
		
		centralData->dist2wp_sqr = vector_norm_sqr(rel_pos);
	}
	
	if (centralData->dist2wp_sqr < 3.0f)
	{
		centralData->critical_next_state = false;
		switch (centralData->critical_behavior)
		{
			case CLIMB_TO_SAFE_ALT:
			dbg_print("Critical State! Flying to home waypoint.\n");
			centralData->critical_behavior = FLY_TO_HOME_WP;
			break;
			case FLY_TO_HOME_WP:
			dbg_print("Critical State! Performing critical landing.\n");
			centralData->critical_behavior = CRITICAL_LAND;
			break;
			case CRITICAL_LAND:
			dbg_print("Critical State! Landed, switching off motors, Emergency mode.\n");
			centralData->critical_landing = true;
			break;
		}
	}
}

void auto_landing()
{
	float rel_pos[3];
	uint8_t i;
	
	local_coordinates_t local_position;
	
	switch(centralData->auto_landing_enum)
	{
		case DESCENT_TO_SMALL_ALTITUDE:
			local_position = centralData->position_estimator.localPosition;
			local_position.pos[Z] = -2.0f;
			
			waypoint_hold_init(local_position);
			break;
		case DESCENT_TO_GND:
			local_position = centralData->position_estimator.localPosition;
			local_position.pos[Z] = 0.0f;
			
			waypoint_hold_init(local_position);
			break;
	}
	
	for (i=0;i<3;i++)
	{
		rel_pos[i] = centralData->waypoint_critical_coordinates.pos[i] - centralData->position_estimator.localPosition.pos[i];
	}
	centralData->dist2wp_sqr = vector_norm_sqr(rel_pos);
	
	if (centralData->dist2wp_sqr < 0.5f)
	{
		switch(centralData->auto_landing_enum)
		{
			case DESCENT_TO_SMALL_ALTITUDE:
				dbg_print("Automatic-landing: descent_to_GND\n");
				centralData->critical_behavior = FLY_TO_HOME_WP;
				break;
			case DESCENT_TO_GND:
				
				break;
		}
	}
}

void continueToNextWaypoint()
{
	if ((centralData->number_of_waypoints>0)&&(!centralData->waypoint_set))
	{
		centralData->waypoint_list[centralData->current_waypoint_count].current = 0;
		
		dbg_print("Continuing towards waypoint Nr");
		
		if (centralData->current_waypoint_count == (centralData->number_of_waypoints-1))
		{
			centralData->current_waypoint_count = 0;
		}
		else
		{
			centralData->current_waypoint_count++;
		}
		dbg_print_num(centralData->current_waypoint_count,10);
		dbg_print("\n");
		centralData->waypoint_list[centralData->current_waypoint_count].current = 1;
		centralData->current_waypoint = centralData->waypoint_list[centralData->current_waypoint_count];
		centralData->waypoint_coordinates = set_waypoint_from_frame(centralData->current_waypoint,centralData->position_estimator.localPosition.origin);
		
		mavlink_msg_mission_current_send(MAVLINK_COMM_0,centralData->current_waypoint_count);
		
		centralData->waypoint_set = true;
	}
	else
	{
		dbg_print("Not ready to switch to next waypoint. Either no waypoint loaded or flying towards one\n");
	}
}

void set_circle_scenario(waypoint_struct waypoint_list[], uint16_t* number_of_waypoints, float circle_radius, float num_of_vhc)
{
	float angle_step = 2.0 * PI / num_of_vhc;
	
	waypoint_struct waypoint;
	
	local_coordinates_t waypoint_transfo;
	global_position_t waypoint_global;
	
	*number_of_waypoints = 2;
	centralData->current_waypoint_count = -1;
	
	waypoint_transfo.origin = centralData->position_estimator.localPosition.origin;
	
	// Start waypoint
	waypoint_transfo.pos[X] = circle_radius * cos(angle_step * (mavlink_system.sysid-1));
	waypoint_transfo.pos[Y] = circle_radius * sin(angle_step * (mavlink_system.sysid-1));
	waypoint_transfo.pos[Z] = -20.0f;
	waypoint_global = local_to_global_position(waypoint_transfo);
	
	dbg_print("Circle departure(x100): (");
	dbg_print_num(waypoint_transfo.pos[X]*100.0f,10);
	dbg_print(", ");
	dbg_print_num(waypoint_transfo.pos[Y]*100.0f,10);
	dbg_print(", ");
	dbg_print_num(waypoint_transfo.pos[Z]*100.0f,10);
	dbg_print("). For system:");
	dbg_print_num(mavlink_system.sysid,10);
	dbg_print(".\n");
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
	waypoint.param4 = rad_to_deg(calc_smaller_angle(PI + angle_step * (mavlink_system.sysid-1))); // Desired yaw angle at MISSION (rotary wing)
	
	waypoint_list[0] = waypoint;
	
	// End waypoint
	waypoint_transfo.pos[X] = circle_radius * cos(angle_step * (mavlink_system.sysid-1) + PI);
	waypoint_transfo.pos[Y] = circle_radius * sin(angle_step * (mavlink_system.sysid-1) + PI);
	waypoint_transfo.pos[Z] = -20.0f;
	waypoint_global = local_to_global_position(waypoint_transfo);
	
	dbg_print("Circle destination(x100): (");
	dbg_print_num(waypoint_transfo.pos[X]*100.0f,10);
	dbg_print(", ");
	dbg_print_num(waypoint_transfo.pos[Y]*100.0f,10);
	dbg_print(", ");
	dbg_print_num(waypoint_transfo.pos[Z]*100.0f,10);
	dbg_print("). For system:");
	dbg_print_num(mavlink_system.sysid,10);
	dbg_print(".\n");
	
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
	
	waypoint_list[1] = waypoint;
	
	centralData->waypoint_set = false;
}
/*
void set_stream_scenario(waypoint_struct waypoint_list[], uint16_t* number_of_waypoints, float circle_radius, float num_of_vhc)
{
	waypoint_struct waypoint;
	
	// TODO: Add code here :)
}
*/
