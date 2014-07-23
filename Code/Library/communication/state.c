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
#include "print_util.h"

void state_init(state_t *state, state_t* state_config, const analog_monitor_t* analog_monitor, mavlink_stream_t* mavlink_stream, mavlink_message_handler_t *message_handler)
{
	state->analog_monitor = analog_monitor;
	state->mavlink_stream = mavlink_stream;
	
	state->autopilot_type = state_config->autopilot_type;
	state->autopilot_name = state_config->autopilot_name;
	
	state->mav_state = state_config->mav_state;
	state->mav_mode = state_config->mav_mode;
	
	state->simulation_mode = state_config->simulation_mode;
	
	state->mav_mode_previous = state->mav_mode;
	
	state->simulation_mode_previous = state->simulation_mode;
	
	if (state->simulation_mode == SIMULATION_MODE)
	{
		state->mav_mode |= MAV_MODE_FLAG_HIL_ENABLED;
	}
	else
	{
		state->mav_mode |= !MAV_MODE_FLAG_HIL_ENABLED;
	}
	
	// Add callbacks for onboard parameters requests
	mavlink_message_handler_msg_callback_t callback;
	
	callback.message_id 	= MAVLINK_MSG_ID_SET_MODE; // 11
	callback.sysid_filter 	= MAVLINK_BASE_STATION_ID;
	callback.compid_filter 	= MAV_COMP_ID_ALL;
	callback.function 		= (mavlink_msg_callback_function_t)	&state_set_mav_mode;
	callback.module_struct 	= (handling_module_struct_t)		state;
	mavlink_message_handler_add_msg_callback( message_handler, &callback );
	
	print_util_dbg_print("State initialized.\n");
}

task_return_t state_send_heartbeat(state_t* state)
{
	mavlink_message_t msg;
	const mavlink_stream_t* mavlink_stream = state->mavlink_stream;
	mavlink_msg_heartbeat_pack(	mavlink_stream->sysid,
								mavlink_stream->compid,
								&msg,
								state->autopilot_type, 
								state->autopilot_name, 
								state->mav_mode, 
								state->simulation_mode, 
								state->mav_state);
	mavlink_stream_send(mavlink_stream, &msg);
	
	return TASK_RUN_SUCCESS;
}

task_return_t state_send_status(state_t* state)
{
	float battery_voltage = state->analog_monitor->avg[ANALOG_RAIL_10];		// bat voltage (mV), actual battery pack plugged to the board
	float battery_remaining = state->analog_monitor->avg[ANALOG_RAIL_11] / 12.4f * 100.0f;
	
	const mavlink_stream_t* mavlink_stream = state->mavlink_stream;
	mavlink_message_t msg;
	mavlink_msg_sys_status_pack(mavlink_stream->sysid,
								mavlink_stream->compid,
								&msg, 
								state->sensor_present, 						// sensors present
								state->sensor_enabled, 						// sensors enabled
								state->sensor_health, 						// sensors health
								0,                  									// load
								(int32_t)(1000.0f * battery_voltage), 					// bat voltage (mV)
								0,               										// current (mA)
								battery_remaining,										// battery remaining
								0, 0,  													// comms drop, comms errors
								0, 0, 0, 0);        									// autopilot specific errors
	mavlink_stream_send(mavlink_stream, &msg);

	return TASK_RUN_SUCCESS;
}

void state_set_mav_mode(state_t* state, mavlink_received_t* rec)
{
	mavlink_set_mode_t packet;
	
	mavlink_msg_set_mode_decode(&rec->msg,&packet);
	
	// Check if this message is for this system and subsystem
	// No component ID in mavlink_set_mode_t so no control
	if ((uint8_t)packet.target_system == (uint8_t)mavlink_system.sysid)
	{
		print_util_dbg_print("base_mode:");
		print_util_dbg_print_num(packet.base_mode,10);
		print_util_dbg_print(", custom mode:");
		print_util_dbg_print_num(packet.custom_mode,10);
		print_util_dbg_print("\n");

		if (state->simulation_mode == REAL_MODE)
		{
			switch(packet.base_mode)
			{
				case MAV_MODE_STABILIZE_DISARMED:
				case MAV_MODE_GUIDED_DISARMED:
				case MAV_MODE_AUTO_DISARMED:
					state->mav_state = MAV_STATE_STANDBY;
					state->mav_mode = MAV_MODE_MANUAL_DISARMED;
					break;
				
				case MAV_MODE_MANUAL_ARMED:
					//if (remote_controller_get_thrust_from_remote()<-0.95f)
					{
						state->mav_state = MAV_STATE_ACTIVE;
						state->mav_mode = MAV_MODE_MANUAL_ARMED;
					}
					break;
			}
		}
		else
		{
			switch(packet.base_mode)
			{
				case MAV_MODE_STABILIZE_DISARMED:
				case MAV_MODE_GUIDED_DISARMED:
				case MAV_MODE_AUTO_DISARMED:
					state->mav_state = MAV_STATE_STANDBY;
					state->mav_mode = MAV_MODE_MANUAL_DISARMED;
					break;
				case MAV_MODE_MANUAL_ARMED:
					state->mav_state = MAV_STATE_ACTIVE;
					state->mav_mode = MAV_MODE_MANUAL_ARMED;
					break;
				case MAV_MODE_STABILIZE_ARMED:
					state->mav_state = MAV_STATE_ACTIVE;
					state->mav_mode = MAV_MODE_STABILIZE_ARMED;
					break;
				case MAV_MODE_GUIDED_ARMED:
					state->mav_state = MAV_STATE_ACTIVE;
					state->mav_mode = MAV_MODE_GUIDED_ARMED;
					break;
				case MAV_MODE_AUTO_ARMED:
					state->mav_state = MAV_STATE_ACTIVE;
					state->mav_mode = MAV_MODE_AUTO_ARMED;
					break;
			}
		}
	}
}

bool state_test_if_in_mode(state_t *state, uint8_t mav_mode)
{
	return (state->mav_mode == (mav_mode + (state->mav_mode & MAV_MODE_FLAG_HIL_ENABLED)));
}

void state_enable_mode(state_t *state, mav_flag_t mav_mode_flag)
{
	state->mav_mode |= mav_mode_flag;
	state->mav_mode_previous = state->mav_mode;
}

void state_disable_mode(state_t *state, mav_flag_t mav_mode_flag)
{
	state->mav_mode |= !mav_mode_flag;
	state->mav_mode_previous = state->mav_mode;
}

bool state_test_if_in_flag_mode(const state_t *state, mav_flag_t mav_mode_flag)
{
	return (state->mav_mode & mav_mode_flag);
}

bool state_test_if_first_time_in_mode(state_t *state, mav_mode_t mav_mode)
{
	return !(state->mav_mode_previous == state->mav_mode);
}

void state_set_new_mode(state_t *state, mav_mode_t mav_mode)
{
	state->mav_mode = mav_mode + (state->mav_mode & MAV_MODE_FLAG_HIL_ENABLED);
}