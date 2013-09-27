/*
 * mavlink_actions.h
 *
 * This file interfaces between the various parts of the system and the MAVlink downstream
 * Put all commands here that collect data and can be scheduled for transmission
 *
 * Created: 21/03/2013 01:00:46
 *  Author: Felix Schill
 */ 


#ifndef MAVLINK_ACTIONS_H_
#define MAVLINK_ACTIONS_H_

void add_PID_parameters(void);

void init_mavlink_actions(void);


#endif /* MAVLINK_ACTIONS_H_ */