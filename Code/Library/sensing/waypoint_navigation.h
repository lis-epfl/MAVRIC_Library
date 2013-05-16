/*
 * waypoint_navigation.h
 *
 * Created: May 16 2013 17:54:34
 *  Author: Nicolas
 */ 

#ifndef WAYPOINT_NAVIGATION__
#define WAYPOINT_NAVIGATION__

#define MAX_WAYPOINTS 10

#include "mavlink_stream.h"

typedef struct {
	uint8_t frame;
	uint16_t command;
	uint8_t current;
	uint8_t autocontinue;
	float param1;
	float param2;
	float param3;
	float param4;
	float x;
	float y;
	float z;
}waypoint_struct;

void send_count(Mavlink_Received_t* rec, uint16_t num_of_waypoint);
void send_waypoint(Mavlink_Received_t* rec, waypoint_struct waypoint);


#endif // WAYPOINT_NAVIGATION__