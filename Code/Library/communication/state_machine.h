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
#include "mavlink_waypoint_handler.h"
#include "simulation.h"

typedef struct 
{
	uint8_t channel_switches;							///< State of the switches of the remote
	signal_quality_t rc_check;							///< State of the remote (receiving signal or not)
	int8_t motor_state;									///< State of the motors to switch on and off
	bool use_mode_from_remote;

	mavlink_waypoint_handler_t* waypoint_handler;
	state_t* state;
	simulation_model_t *sim_model;
	remote_t* remote;
} state_machine_t;

typedef struct  
{
	state_machine_t state_machine;
}state_machine_conf_t;

void state_machine_init(state_machine_t *state_machine, const state_machine_conf_t* state_machine_conf, state_t* state, mavlink_waypoint_handler_t* waypoint_handler, simulation_model_t *sim_model, remote_t* remote);


/**
 * \brief            Updates the state and mode of the UAV
 */
task_return_t state_machine_set_mav_mode_n_state(state_machine_t* state_machine);


void state_machine_update(state_machine_t* state_machine);


#ifdef __cplusplus
}
#endif

#endif // STATE_MACHINE_H_