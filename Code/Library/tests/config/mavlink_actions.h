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

void mavlink_actions_init(void);

static inline void mavlink_actions_handle_specific_messages (mavlink_received_t* rec) {}

void mavlink_actions_receive_message_long(mavlink_received_t* rec);

#endif /* MAVLINK_ACTIONS_H_ */