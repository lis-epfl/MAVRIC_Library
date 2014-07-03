/*
 * mavlink_actions.h
 *
 * This file interfaces between the various parts of the system and the MAVlink downstream
 * Put all commands here that collect data and can be scheduled for transmission
 *
 * Created: 21/03/2013 01:00:46
 *  Author: sfx
 */ 


#ifndef MAVLINK_ACTIONS_H_
#define MAVLINK_ACTIONS_H_

#include "mavlink_stream.h"


void mavlink_actions_init();

void mavlink_send_radar() ;
void mavlink_send_radar_raw();

void mavlink_actions_handle_specific_messages (Mavlink_Received_t* rec);

#endif /* MAVLINK_ACTIONS_H_ */