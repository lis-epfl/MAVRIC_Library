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
 * \file state.c
 *
 *  Place where the state structure is defined
 */


#include "state.h"
#include "mavlink/include/maveric/mavlink.h"

void state_init(state_structure_t *state_structure, uint8_t autopilot_type, uint8_t autopilot_name, uint8_t mav_state, uint8_t mav_mode, HIL_mode simu_mode)
{
	state_structure->autopilot_type = autopilot_type;
	state_structure->autopilot_name = autopilot_name;
	
	state_structure->mav_state = mav_state;
	state_structure->mav_mode = mav_mode;
	
	state_structure->simulation_mode = simu_mode;
	
	state_structure->mav_mode_previous = state_structure->mav_mode;
	state_structure->mav_state_previous = state_structure->mav_state;
	
	state_structure->simulation_mode_previous = state_structure->simulation_mode;
}

//task_return_t mavlink_telemetry_send_heartbeat(state_structure_t* state_structure)
//{
	////mavlink_msg_heartbeat_send(MAVLINK_COMM_0, state_structure->autopilot_type, state_structure->autopilot_name, state_structure->mav_mode, 0, state_structure->mav_state);
	//
	//return TASK_RUN_SUCCESS;
//}