/*
 * waypoint_navigation.h
 *
 * Created: May 16 2013 17:54:34
 *  Author: Nicolas
 */ 

#ifndef WAYPOINT_NAVIGATION__
#define WAYPOINT_NAVIGATION__

#include "mavlink_stream.h"
#include "stdbool.h"

#define MAX_WAYPOINTS 10

/*
* N.B.: Frames, MAV_CMD_NAV are defined in "maveric.h"
*/

typedef struct {
	uint8_t frame;
	uint16_t wp_id;
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

bool waypoint_sending;
bool waypoint_receiving;

//int num_of_waypoint;
int sending_wp_num;
int waypoint_request_number;

uint16_t num_waypoint_onboard;

uint32_t start_timeout;
uint32_t timeout_max_wp;

void init_waypoint_list(waypoint_struct waypoint_list[], uint16_t* number_of_waypoints);

void send_count(Mavlink_Received_t* rec, uint16_t num_of_waypoint);
void send_waypoint(Mavlink_Received_t* rec, waypoint_struct waypoint[], uint16_t num_of_waypoint);
void receive_ack_msg(Mavlink_Received_t* rec);

void receive_count(Mavlink_Received_t* rec, uint16_t* number_of_waypoints);
void receive_waypoint(Mavlink_Received_t* rec,  waypoint_struct waypoint_list[], uint16_t number_of_waypoints);
void set_current_wp(Mavlink_Received_t* rec,  waypoint_struct* waypoint_list[], uint16_t num_of_waypoint);
void clear_waypoint_list(Mavlink_Received_t* rec,  uint16_t* number_of_waypoints);

void set_mav_mode(Mavlink_Received_t* rec, uint8_t* board_mav_mode, uint8_t* board_mav_state);
void receive_message_long(Mavlink_Received_t* rec);

void control_time_out_waypoint_msg(uint16_t* num_of_waypoint);

#endif // WAYPOINT_NAVIGATION__