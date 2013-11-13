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

void add_PID_parameters(void);

void init_mavlink_actions(void);

void handle_specific_messages (Mavlink_Received_t* rec);
void receive_message_long(Mavlink_Received_t* rec);

#endif /* MAVLINK_ACTIONS_H_ */