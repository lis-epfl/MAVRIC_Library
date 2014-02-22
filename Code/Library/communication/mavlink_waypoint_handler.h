/*
 * mavlink_waypoint_handler.h
 *
 * Created: May 16 2013 17:54:34
 *  Author: Nicolas
 */ 

#ifndef MAVLINK_WAYPOINT_HANDLER__
#define MAVLINK_WAYPOINT_HANDLER__

#include "mavlink_stream.h"
#include "stdbool.h"
#include "coord_conventions.h"

#define MAX_WAYPOINTS 10

#ifdef __cplusplus
extern "C" {
#endif

/*
* N.B.: Frames, MAV_CMD_NAV are defined in "maveric.h"
*/

typedef struct {
	uint8_t frame;
	uint16_t wp_id;
	uint8_t current;
	uint8_t autocontinue;
	float param1;   // hold time in seconds
	float param2;   // acceptance radius
	float param3;   // loiter radius
	float param4;   // desired heading at waypoint
	double x;
	double y;
	double z;
	
}waypoint_struct;

int sending_wp_num;
int waypoint_request_number;

uint16_t num_waypoint_onboard;

uint32_t start_timeout;
uint32_t timeout_max_wp;

void init_waypoint_handler();
void init_wp();
void init_waypoint_list(waypoint_struct waypoint_list[], uint16_t* number_of_waypoints);

void send_count(Mavlink_Received_t* rec, uint16_t num_of_waypoint, bool* waypoint_receiving, bool * waypoint_sending);
void send_waypoint(Mavlink_Received_t* rec, waypoint_struct waypoint[], uint16_t num_of_waypoint, bool* waypoint_sending);
void receive_ack_msg(Mavlink_Received_t* rec, bool* waypoint_sending);

void receive_count(Mavlink_Received_t* rec, uint16_t* number_of_waypoints, bool* waypoint_receiving, bool* waypoint_sending);
void receive_waypoint(Mavlink_Received_t* rec,  waypoint_struct waypoint_list[], uint16_t number_of_waypoints, bool* waypoint_receiving);
void set_current_wp(Mavlink_Received_t* rec,  waypoint_struct waypoint_list[], uint16_t num_of_waypoint);
void clear_waypoint_list(Mavlink_Received_t* rec,  uint16_t* number_of_waypoints, bool* waypoint_set);
void set_home(Mavlink_Received_t* rec);

void set_mav_mode(Mavlink_Received_t* rec, uint8_t* board_mav_mode, uint8_t* board_mav_state, uint8_t sim_mode);

void control_time_out_waypoint_msg(uint16_t* num_of_waypoint, bool* waypoint_receiving, bool* waypoint_sending);

local_coordinates_t set_waypoint_from_frame(waypoint_struct current_wp, global_position_t origin);
void wp_hold_init();
void wp_take_off();

void waypoint_hold_position_handler();
void waypoint_navigation_handler();
void waypoint_critical_handler();

#ifdef __cplusplus
}
#endif

#endif // MAVLINK_WAYPOINT_HANDLER__