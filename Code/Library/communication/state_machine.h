/**
 * \page The MAV'RIC License
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 * 
 * Laboratory of Intelligent Systems, EPFL
 */


/**
 * \file state_machine.h
 *
 * Definition of the tasks executed on the autopilot
 */ 


#ifndef STATE_MACHINE_H_
#define STATE_MACHINE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "scheduler.h"
#include "state.h"
#include "waypoint_handler.h"
#include "navigation.h"

typedef struct 
{
	state_structure_t* state_structure;
	mavlink_waypoint_handler_t* waypoint_handler;
	navigation_t *navigation_data;
}state_machine_t;

void state_machine_init(state_machine_t *state_machine, state_structure_t* state_structure, mavlink_waypoint_handler_t* waypoint_handler, navigation_t* navigation_data);



/**
 * \brief            Updates the state and mode of the UAV
 */
task_return_t state_machine_set_mav_mode_n_state(void* arg);








#ifdef __cplusplus
}
#endif

#endif // STATE_MACHINE_H_