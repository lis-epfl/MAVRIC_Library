/*
 * waypoint_navigation.c
 *
 * Created: May 16 2013 17:54:34
 *  Author: Nicolas
 */ 

#include "waypoint_navigation.h"


void send_count(Mavlink_Received_t* rec, uint16_t num_of_waypoint)
{
	mavlink_mission_request_list_t packet;
	mavlink_msg_mission_request_list_decode(&rec->msg,&packet);
	
	// Check if this message is for this system and subsystem
	if ((uint8_t)packet.target_system == (uint8_t)mavlink_system.sysid
	&& (uint8_t)packet.target_component == (uint8_t)mavlink_system.compid)
	{	
		mavlink_msg_mission_count_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,num_of_waypoint);
	}
}

void send_waypoint(Mavlink_Received_t* rec, waypoint_struct waypoint)
{
	mavlink_mission_request_t packet;
	mavlink_msg_mission_request_list_decode(rec,&packet);
	// Check if this message is for this system and subsystem
	if ((uint8_t)packet.target_system == (uint8_t)mavlink_system.sysid
	&& (uint8_t)packet.target_component == (uint8_t)mavlink_system.compid) 
	{
		mavlink_msg_mission_item_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,packet.seq,waypoint.frame,waypoint.command,
		waypoint.current,waypoint.autocontinue,waypoint.param1,waypoint.param2,waypoint.param3,waypoint.param4,
		waypoint.x,waypoint.y,waypoint.z);
	}
}