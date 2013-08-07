/*
 * waypoint_navigation.c
 *
 * Created: May 16 2013 17:54:34
 *  Author: Nicolas
 */ 

#include "waypoint_navigation.h"
#include "print_util.h"
#include "spektrum.h"

void init_waypoint_list(waypoint_struct waypoint_list[], uint16_t* number_of_waypoints)
{
	// Visit https://code.google.com/p/ardupilot-mega/wiki/MAVLink to have a description of all messages (or common.h)
	waypoint_struct waypoint;
	*number_of_waypoints = 4;
	
	
	// Set home waypoint
	waypoint.autocontinue = 1;
	waypoint.current = 1;
	waypoint.frame = 0; // 0:Global, 1:local NED, 2:mission, 3:global rel alt, 4: local ENU
	waypoint.wp_id = MAV_CMD_DO_SET_HOME;

	
	waypoint.param1 = 0;
	waypoint.param2 = 520; // altitude
	waypoint.param3 = 465185536 / 1.0e7f; // lat converted to deg
	waypoint.param4 = 65670560 / 1.0e7f; // long converted to deg
	
	waypoint_list[0] = waypoint;
	
	// Set nav waypoint
	waypoint.autocontinue = 1;
	waypoint.current = 0;
	waypoint.frame = 0;
	waypoint.wp_id = MAV_CMD_NAV_WAYPOINT;
	
	waypoint.x = 465185536 / 1.0e7f; // convert to deg
	waypoint.y = 65670560 / 1.0e7f; // convert to deg
	waypoint.z = 520; //m
	
	waypoint.param1 = 10; // Hold time in decimal seconds
	waypoint.param2 = 15; // Acceptance radius in meters
	waypoint.param3 = 0; //  0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
	waypoint.param4 = 90; // Desired yaw angle at MISSION (rotary wing)
	
	waypoint_list[1] = waypoint;
	
	// Set nav waypoint
	waypoint.autocontinue = 1;
	waypoint.current = 0;
	waypoint.frame = 0;
	waypoint.wp_id = MAV_CMD_NAV_WAYPOINT;
	
	waypoint.x = 465186816 / 1.0e7f; // convert to deg
	waypoint.y = 65659084 / 1.0e7f; // convert to deg
	waypoint.z = 520; //m
	
	waypoint.param1 = 10; // Hold time in decimal seconds
	waypoint.param2 = 15; // Acceptance radius in meters
	waypoint.param3 = 0; //  0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
	waypoint.param4 = 90; // Desired yaw angle at MISSION (rotary wing)
	
	waypoint_list[2] = waypoint;
	
	// Set nav waypoint
	waypoint.autocontinue = 1;
	waypoint.current = 0;
	waypoint.frame = 0;
	waypoint.wp_id = MAV_CMD_NAV_WAYPOINT;

	waypoint.x = 465186816 / 1.0e7f; // convert to deg
	waypoint.y = 65659084 / 1.0e7f; // convert to deg
	waypoint.z = 520; //m

	waypoint.param1 = 10; // Hold time in decimal seconds
	waypoint.param2 = 15; // Acceptance radius in meters
	waypoint.param3 = 0; //  0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
	waypoint.param4 = 90; // Desired yaw angle at MISSION (rotary wing)

	waypoint_list[3] = waypoint;
	
}

void send_count(Mavlink_Received_t* rec, uint16_t num_of_waypoint_)
{
	mavlink_mission_request_list_t packet;
	mavlink_msg_mission_request_list_decode(&rec->msg,&packet);
	
	// Check if this message is for this system and subsystem
	if ((uint8_t)packet.target_system == (uint8_t)mavlink_mission_planner.sysid
	&& (uint8_t)packet.target_component == (uint8_t)mavlink_mission_planner.compid)
	{	
		num_of_waypoint = num_of_waypoint_;
		mavlink_msg_mission_count_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,num_of_waypoint);
		waypoint_sending = true;
		sending_wp_num = 0;
	}
}

void send_waypoint(Mavlink_Received_t* rec, waypoint_struct waypoint[])
{
	if (waypoint_sending)
	{
		mavlink_mission_request_t packet;
		mavlink_msg_mission_request_decode(&rec->msg,&packet);
		// Check if this message is for this system and subsystem
		if ((uint8_t)packet.target_system == (uint8_t)mavlink_mission_planner.sysid
		&& (uint8_t)packet.target_component == (uint8_t)mavlink_mission_planner.compid)
		{
			if (sending_wp_num < num_of_waypoint)
			{
				// mavlink_msg_mission_item_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z)
				mavlink_msg_mission_item_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,packet.seq,
				waypoint[sending_wp_num].frame,   waypoint[sending_wp_num].wp_id,
				waypoint[sending_wp_num].current, waypoint[sending_wp_num].autocontinue,
				waypoint[sending_wp_num].param1,  waypoint[sending_wp_num].param2,       waypoint[sending_wp_num].param3,    waypoint[sending_wp_num].param4,
				waypoint[sending_wp_num].x,       waypoint[sending_wp_num].y,            waypoint[sending_wp_num].z);
			
				sending_wp_num += 1;
			}			
		}
	}	
}

void receive_ack_msg(Mavlink_Received_t* rec)
{
	mavlink_mission_ack_t packet;
	mavlink_msg_mission_ack_decode(&rec->msg, &packet);
	// Check if this message is for this system and subsystem
	if ((uint8_t)packet.target_system == (uint8_t)mavlink_mission_planner.sysid
	&& (uint8_t)packet.target_component == (uint8_t)mavlink_mission_planner.compid)
	{
		waypoint_sending = false;
		sending_wp_num = 0;
	}
}

void receive_count(Mavlink_Received_t* rec, uint16_t* number_of_waypoints)
{
	mavlink_mission_count_t packet;
	mavlink_msg_mission_count_decode(&rec->msg, &packet);
	// Check if this message is for this system and subsystem
	//dbg_print("check msg");
	//dbg_print_num(packet.target_system,10);
	//dbg_print_num(mavlink_mission_planner.sysid,10);
	//dbg_print_num(packet.target_component,10);
	//dbg_print_num(mavlink_mission_planner.compid,10);
	//dbg_print("\n");
	if ((uint8_t)packet.target_system == (uint8_t)mavlink_mission_planner.sysid
	&& (uint8_t)packet.target_component == (uint8_t)mavlink_mission_planner.compid)
	{
		if (packet.count > MAX_WAYPOINTS)
		{
			packet.count = MAX_WAYPOINTS;
		}
		*number_of_waypoints = packet.count;
		
		waypoint_receiving   = true;
		waypoint_sending     = false;
		waypoint_request_number = 0;
	}
	mavlink_msg_mission_request_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,waypoint_request_number);
}

void receive_waypoint(Mavlink_Received_t* rec,  waypoint_struct waypoint_list[], uint16_t number_of_waypoints)
{
	mavlink_mission_item_t packet;
	mavlink_msg_mission_item_decode(&rec->msg,&packet);
	// Check if this message is for this system and subsystem
	if ((uint8_t)packet.target_system == (uint8_t)mavlink_mission_planner.sysid
	&& (uint8_t)packet.target_component == (uint8_t)mavlink_mission_planner.compid)
	{
		waypoint_struct new_waypoint;
		
		new_waypoint.wp_id = packet.command;
		
		new_waypoint.x = 1.0e7f * packet.x; // longitude converted to e7
		new_waypoint.y = 1.0e7f * packet.y; // latitude converted to e7
		new_waypoint.z = 1.0e2f * packet.z; // altitude converted to cm
		
		new_waypoint.autocontinue = packet.autocontinue;
		new_waypoint.frame = packet.frame;
		
		new_waypoint.param1 = packet.param1;
		new_waypoint.param2 = packet.param2;
		new_waypoint.param3 = packet.param3;
		new_waypoint.param4 = packet.param4;
		
		//dbg_print("New waypoint received ");
		//dbg_print("(");
 		//dbg_print_num(new_waypoint.x,10);
 		//dbg_print(", ");
 		//dbg_print_num(new_waypoint.y,10);
 		//dbg_print(", ");
 		//dbg_print_num(new_waypoint.z,10);
 		//dbg_print(") Autocontinue:");
 		//dbg_print_num(new_waypoint.autocontinue,10);
 		//dbg_print(" Frame:");
 		//dbg_print_num(new_waypoint.frame,10);
 		//dbg_print(" Current :");
 		//dbg_print_num(packet.current,10);
 		//dbg_print(" Seq :");
 		//dbg_print_num(packet.seq,10);
		//dbg_print(" command id :");
		//dbg_print_num(packet.command,10);
		//dbg_print(" request num :");
		//dbg_print_num(waypoint_request_number,10);
		//dbg_print(" is it receiving :");
		//dbg_print_num(waypoint_receiving,10); // boolean value
		//dbg_print("\n");
		
		//switch(packet.command)
		//{
			//case MAV_CMD_NAV_LOITER_TURNS:
			//case MAV_CMD_DO_SET_HOME:
			//new_waypoint.param1 = packet.param1;
			//break;
//
			//case MAV_CMD_NAV_ROI:
			//new_waypoint.param1 = packet.param1;                                    // MAV_ROI (aka roi mode) is held in wp's parameter but we actually do nothing with it because we only support pointing at a specific location provided by x,y and z parameters
			//break;
//
			//case MAV_CMD_CONDITION_YAW:
			//new_waypoint.param1 = packet.param3;
			//new_waypoint.alt = packet.param1;
			//new_waypoint.lat = packet.param2;
			//new_waypoint.lon = packet.param4;
			//break;
//
			//case MAV_CMD_NAV_TAKEOFF:
			//new_waypoint.param1 = 0;
			//break;
//
			//case MAV_CMD_CONDITION_CHANGE_ALT:
			//new_waypoint.param1 = packet.param1 * 100;
			//break;
//
			//case MAV_CMD_NAV_LOITER_TIME:
			//new_waypoint.param1 = packet.param1;                                    // APM loiter time is in ten second increments
			//break;
//
			//case MAV_CMD_CONDITION_DELAY:
			//case MAV_CMD_CONDITION_DISTANCE:
			//new_waypoint.lat = packet.param1;
			//break;
//
			//case MAV_CMD_DO_JUMP:
			//new_waypoint.lat = packet.param2;
			//new_waypoint.param1  = packet.param1;
			//break;
//
			//case MAV_CMD_DO_REPEAT_SERVO:
			//new_waypoint.lon = packet.param4;
			//case MAV_CMD_DO_REPEAT_RELAY:
			//case MAV_CMD_DO_CHANGE_SPEED:
			//new_waypoint.lat = packet.param3;
			//new_waypoint.alt = packet.param2;
			//new_waypoint.param1 = packet.param1;
			//break;
//
			//case MAV_CMD_NAV_WAYPOINT:
			//new_waypoint.param1 = packet.param1;
			//break;
//
			//case MAV_CMD_DO_SET_PARAMETER:
			//case MAV_CMD_DO_SET_RELAY:
			//case MAV_CMD_DO_SET_SERVO:
			//new_waypoint.alt = packet.param2;
			//new_waypoint.param1 = packet.param1;
			//break;
		//}
		
		if(packet.current == 2) {                                               //current = 2 is a flag to tell us this is a "guided mode" waypoint and not for the mission
			// switch to guided mode
			//set_mode(GUIDED);

			// set wp_nav's destination
			//wp_nav.set_destination(pv_location_to_vector(tell_command));

			// verify we received the command
			mavlink_msg_mission_ack_send(MAVLINK_COMM_0, rec->msg.sysid,rec->msg.compid, 0);

		} else if(packet.current == 3){                                    //current = 3 is a flag to tell us this is a alt change only

			// add home alt if needed
			//if (new_waypoint.options & MASK_OPTIONS_RELATIVE_ALT) 
			//{
			//	new_waypoint.alt += home.alt;
			//}

			// To-Do: update target altitude for loiter or waypoint controller depending upon nav mode
			// similar to how do_change_alt works
			//wp_nav.set_desired_alt(new_waypoint.alt);

			// verify we received the command
			mavlink_msg_mission_ack_send(MAVLINK_COMM_0, rec->msg.sysid,rec->msg.compid, 0);

		} else {
			// Check if receiving waypoints
			if (waypoint_receiving){

				// check if this is the requested waypoint
				if (packet.seq == waypoint_request_number)
				{
					dbg_print("Receiving good waypoint, number ");
					dbg_print_num(waypoint_request_number+1,10);
					dbg_print(" of ");
					dbg_print_num(number_of_waypoints,10);
					dbg_print(" \n");
					
				    waypoint_list[waypoint_request_number] = new_waypoint;
					waypoint_request_number++;
					
					if (waypoint_request_number == number_of_waypoints) 
					{
						
						uint8_t type = 0;                         // ok (0), error(1)
						mavlink_msg_mission_ack_send(MAVLINK_COMM_0,rec->msg.sysid,mavlink_mission_planner.compid,type);
						mavlink_msg_mission_ack_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,type);
						mavlink_msg_mission_ack_send(MAVLINK_COMM_0,rec->msg.sysid,0,type);
						mavlink_msg_mission_ack_send(MAVLINK_COMM_0,rec->msg.sysid,mavlink_system.compid,type);

						dbg_print("flight plan received!\n");
						waypoint_receiving = false;
					}else{
						mavlink_msg_mission_request_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,waypoint_request_number);
					}
				}
			}else{
				uint8_t type = 1;                         // ok (0), error(1)
				dbg_print("Ack error!");
				mavlink_msg_mission_ack_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,type);
			}				
		}		
	}			
}		

void set_current_wp(Mavlink_Received_t* rec,  waypoint_struct* waypoint_list[])
{
	mavlink_mission_set_current_t packet;
	mavlink_msg_mission_set_current_decode(&rec->msg,&packet);
	// Check if this message is for this system and subsystem
	if ((uint8_t)packet.target_system == (uint8_t)mavlink_mission_planner.sysid
	&& (uint8_t)packet.target_component == (uint8_t)mavlink_mission_planner.compid)
	{
		int i;
		for (i=0;i<MAX_WAYPOINTS;i++)
		{
			waypoint_list[i]->current = 0;
		}
		waypoint_list[packet.seq]->current = 1;
		mavlink_msg_mission_current_send(MAVLINK_COMM_0,waypoint_list[packet.seq]->current);
	}
}

void clear_waypoint_list(Mavlink_Received_t* rec,  waypoint_struct* waypoint_list[])
{
	mavlink_mission_clear_all_t packet;
	mavlink_msg_mission_clear_all_decode(&rec->msg,&packet);
	// Check if this message is for this system and subsystem
	if ((uint8_t)packet.target_system == (uint8_t)mavlink_mission_planner.sysid
	&& (uint8_t)packet.target_component == (uint8_t)mavlink_mission_planner.compid)
	{
		// TODO: clear array
		mavlink_msg_mission_ack_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,0);
	}		
}

void set_mav_mode(Mavlink_Received_t* rec, uint8_t* board_mav_mode, uint8_t* board_mav_state)
{
	mavlink_set_mode_t packet;
	mavlink_msg_set_mode_decode(&rec->msg,&packet);
	// Check if this message is for this system and subsystem
	if ((uint8_t)packet.target_system == (uint8_t)mavlink_mission_planner.sysid)
	{
		//dbg_print("base_mode:");
		//dbg_print_num(packet.base_mode,10);
		//dbg_print(", custom mode:");
		//dbg_print_num(packet.custom_mode,10);
		//dbg_print("\n");
		
		switch(packet.base_mode)
		{
			case MAV_MODE_STABILIZE_DISARMED:
			case MAV_MODE_GUIDED_DISARMED:
			case MAV_MODE_AUTO_DISARMED:
				*board_mav_state = MAV_STATE_STANDBY;
				*board_mav_mode = MAV_MODE_MANUAL_DISARMED;
				break;
			case MAV_MODE_MANUAL_ARMED:
				if ((getChannel(S_THROTTLE)/350.0)<-0.95)
				{
					*board_mav_state = MAV_STATE_ACTIVE;
					*board_mav_mode = MAV_MODE_STABILIZE_ARMED;
				}
				break;
		}
		
		
		
	}
}

void receive_message_long(Mavlink_Received_t* rec)
{
	mavlink_command_long_t packet;
	mavlink_msg_command_long_decode(&rec->msg,&packet);
	// Check if this message is for this system and subsystem
	dbg_print("target_comp:");
	dbg_print_num(packet.target_component,10);
	if ((uint8_t)packet.target_system == (uint8_t)mavlink_mission_planner.sysid
	&&(uint8_t)packet.target_component == (uint8_t)0)
	{
		dbg_print("parameters:");
		dbg_print_num(packet.param1,10);
		dbg_print_num(packet.param2,10);
		dbg_print_num(packet.param3,10);
		dbg_print_num(packet.param4,10);
		dbg_print_num(packet.param5,10);
		dbg_print_num(packet.param6,10);
		dbg_print_num(packet.param7,10);
		dbg_print(", command id:");
		dbg_print_num(packet.command,10);
		dbg_print(", confirmation:");
		dbg_print_num(packet.confirmation,10);
		dbg_print("\n");
	}
}