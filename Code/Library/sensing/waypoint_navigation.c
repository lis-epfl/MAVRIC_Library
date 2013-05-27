/*
 * waypoint_navigation.c
 *
 * Created: May 16 2013 17:54:34
 *  Author: Nicolas
 */ 

#include "waypoint_navigation.h"
#include "print_util.h"

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
			if (sending_wp_num <= num_of_waypoint)
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
	dbg_print("check msg");
	dbg_print_num(packet.target_system,10);
	dbg_print_num(mavlink_mission_planner.sysid,10);
	dbg_print_num(packet.target_component,10);
	dbg_print_num(mavlink_mission_planner.compid,10);
	dbg_print("\n");
	if ((uint8_t)packet.target_system == (uint8_t)mavlink_mission_planner.sysid
	&& (uint8_t)packet.target_component == (uint8_t)mavlink_mission_planner.compid)
	{
		if (packet.count > MAX_WAYPOINTS)
		{
			packet.count = MAX_WAYPOINTS;
		}
		number_of_waypoints = packet.count;
		
		waypoint_receiving   = true;
		waypoint_sending     = false;
		waypoint_request_number = 0;
	}
	mavlink_msg_mission_request_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,waypoint_request_number);
}

void receive_waypoint(Mavlink_Received_t* rec,  waypoint_struct* waypoint_list[], uint16_t number_of_waypoints)
{
	mavlink_mission_item_t packet;
	mavlink_msg_mission_item_decode(&rec->msg,&packet);
	// Check if this message is for this system and subsystem
	if ((uint8_t)packet.target_system == (uint8_t)mavlink_mission_planner.sysid
	&& (uint8_t)packet.target_component == (uint8_t)mavlink_mission_planner.compid)
	{
		waypoint_struct* new_waypoint = waypoint_list[waypoint_request_number];
		new_waypoint->wp_id = packet.command;
		
		new_waypoint->x = 1.0e7f * packet.x; // longitude converted to e7
		new_waypoint->y = 1.0e7f * packet.y; // latitude converted to e7
		new_waypoint->z = 1.0e2f * packet.z; // altitude converted to cm
		
		new_waypoint->autocontinue = packet.autocontinue;
		new_waypoint->frame = packet.frame;
		
		dbg_print("New waypoint received (");
		dbg_print_num(new_waypoint->x,10);
		dbg_print(", ");
		dbg_print_num(new_waypoint->y,10);
		dbg_print(", ");
		dbg_print_num(new_waypoint->z,10);
		dbg_print(")");
		dbg_print_num(new_waypoint->autocontinue,10);
		dbg_print_num(new_waypoint->frame,10);
		dbg_print_num(packet.current,10);
		dbg_print_num(packet.seq,10);
		dbg_print_num(waypoint_request_number,10);
		dbg_print_num(waypoint_receiving,10);
		dbg_print("\n");
		
		//switch(new_waypoint->wp_id)
		//{
			//case MAV_CMD_NAV_LOITER_TURNS:
			//case MAV_CMD_DO_SET_HOME:
			//new_waypoint->param1 = packet.param1;
			//break;
//
			//case MAV_CMD_NAV_ROI:
			//new_waypoint->param1 = packet.param1;                                    // MAV_ROI (aka roi mode) is held in wp's parameter but we actually do nothing with it because we only support pointing at a specific location provided by x,y and z parameters
			//break;
//
			//case MAV_CMD_CONDITION_YAW:
			//new_waypoint->param1 = packet.param3;
			//new_waypoint->alt = packet.param1;
			//new_waypoint->lat = packet.param2;
			//new_waypoint->lon = packet.param4;
			//break;
//
			//case MAV_CMD_NAV_TAKEOFF:
			//new_waypoint->param1 = 0;
			//break;
//
			//case MAV_CMD_CONDITION_CHANGE_ALT:
			//new_waypoint->param1 = packet.param1 * 100;
			//break;
//
			//case MAV_CMD_NAV_LOITER_TIME:
			//new_waypoint->param1 = packet.param1;                                    // APM loiter time is in ten second increments
			//break;
//
			//case MAV_CMD_CONDITION_DELAY:
			//case MAV_CMD_CONDITION_DISTANCE:
			//new_waypoint->lat = packet.param1;
			//break;
//
			//case MAV_CMD_DO_JUMP:
			//new_waypoint->lat = packet.param2;
			//new_waypoint->param1  = packet.param1;
			//break;
//
			//case MAV_CMD_DO_REPEAT_SERVO:
			//new_waypoint->lon = packet.param4;
			//case MAV_CMD_DO_REPEAT_RELAY:
			//case MAV_CMD_DO_CHANGE_SPEED:
			//new_waypoint->lat = packet.param3;
			//new_waypoint->alt = packet.param2;
			//new_waypoint->param1 = packet.param1;
			//break;
//
			//case MAV_CMD_NAV_WAYPOINT:
			//new_waypoint->param1 = packet.param1;
			//break;
//
			//case MAV_CMD_DO_SET_PARAMETER:
			//case MAV_CMD_DO_SET_RELAY:
			//case MAV_CMD_DO_SET_SERVO:
			//new_waypoint->alt = packet.param2;
			//new_waypoint->param1 = packet.param1;
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
			//if (new_waypoint->options & MASK_OPTIONS_RELATIVE_ALT) 
			//{
			//	new_waypoint->alt += home.alt;
			//}

			// To-Do: update target altitude for loiter or waypoint controller depending upon nav mode
			// similar to how do_change_alt works
			//wp_nav.set_desired_alt(new_waypoint->alt);

			// verify we received the command
			mavlink_msg_mission_ack_send(MAVLINK_COMM_0, rec->msg.sysid,rec->msg.compid, 0);

		} else {
			// Check if receiving waypoints (mission upload expected)
			dbg_print("Total new waypoint");
			if (waypoint_receiving){
				//cliSerial->printf("req: %d, seq: %d, total: %d\n", waypoint_request_i,packet.seq, g.command_total.get());

				// check if this is the requested waypoint
				if (packet.seq = waypoint_request_number)
				{
					dbg_print("Receiving good waypoint");
					dbg_print_num(number_of_waypoints,10);
					//if(packet.seq != 0)
					//set_cmd_with_index(tell_command, packet.seq);

					// update waypoint receiving state machine
					//waypoint_timelast_receive = millis();
					//waypoint_timelast_request = 0;
					waypoint_request_number++;

					if (waypoint_request_number > number_of_waypoints) 
					{
						uint8_t type = 0;                         // ok (0), error(1)
						mavlink_msg_mission_ack_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,type);

						dbg_print("flight plan received");
						waypoint_receiving = false;
					}else{
						mavlink_msg_mission_request_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,waypoint_request_number);
					}						
				}
			}else{
				uint8_t type = 1;                         // ok (0), error(1)
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