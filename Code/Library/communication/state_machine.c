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

#include "state_machine.h"
#include "remote_controller.h"
#include "remote_dsm2.h"
#include "led.h"
#include "print_util.h"

/**
 * \brief            	This function does bullshit
 * \details  			1) Switch on/off collision avoidance
 * 						2) Switch on/off the motor
 * 						3) Check the receivers
 * 
 * \param	chanSwitch	The pointer to set the switch mode
 * \param	rc_check	The pointer to the state_machine->state of the remote
 * \param	motorstate	The pointer to the motor state_machine->state
 */
void state_machine_rc_user_channels(state_machine_t* state_machine);

/**
 * \brief	Function to call when the motors should be switched off
 */
void state_machine_switch_off_motors(state_machine_t* state_machine);

void state_machine_init(state_machine_t *state_machine, state_t* state, mavlink_waypoint_handler_t* waypoint_handler, simulation_model_t *sim_model)
{
	state_machine->waypoint_handler = waypoint_handler;
	state_machine->state = state;
	state_machine->sim_model = sim_model;
		
	state_machine->channel_switches = 0;
	state_machine->rc_check = 0;
	state_machine->motor_state = 0;
	
	print_util_dbg_print("State machine initialise.\r");
}

void state_machine_rc_user_channels(state_machine_t* state_machine)
{
	state_machine->rc_check = remote_dsm2_rc_check_receivers();
	
	if (state_machine->rc_check == 1)
	{
		remote_controller_get_channel_mode(&state_machine->channel_switches);
	}
	
	if ((remote_dsm2_rc_get_channel_neutral(RC_TRIM_P3) * RC_SCALEFACTOR) > 0.0f)
	{
		state_machine->state->collision_avoidance = true;
	}
	else
	{
		state_machine->state->collision_avoidance = false;
	}
	
	remote_controller_get_motor_state(&state_machine->motor_state);
}

void state_machine_switch_off_motors(state_machine_t* state_machine)
{
	print_util_dbg_print("Switching off motors!\n");

	state_machine->state->mav_state = MAV_STATE_STANDBY;
	//state_disable_mode(state_machine->state, MAV_MODE_FLAG_SAFETY_ARMED);
	state_set_new_mode(state_machine->state, MAV_MODE_SAFE);
	
	state_machine->state->in_the_air = false;
	
	state_machine->waypoint_handler->auto_landing_behavior = DESCENT_TO_SMALL_ALTITUDE;
	state_machine->waypoint_handler->critical_behavior = CLIMB_TO_SAFE_ALT;
}

task_return_t state_machine_set_mav_mode_n_state(state_machine_t* state_machine)
{
	
	LED_Toggle(LED1);
	
	state_machine_rc_user_channels(state_machine);
	
	if (state_machine->state->remote_active)
	{
		switch(state_machine->state->mav_state)
		{
			case MAV_STATE_BOOT:
				break;

			case MAV_STATE_CALIBRATING:
				break;

			case MAV_STATE_STANDBY:
				if (state_machine->motor_state == 1)
				{
					switch(state_machine->channel_switches)
					{
						case 0:
							print_util_dbg_print("Switching on the motors!\n");

							state_machine->state->reset_position = true;
				
							state_machine->state->nav_plan_active = false;
							state_machine->state->mav_state = MAV_STATE_ACTIVE;
							state_set_new_mode(state_machine->state,MAV_MODE_ATTITUDE_CONTROL);
							break;

						default:
							print_util_dbg_print("Switches not ready, both should be pushed!\n");
							break;
					}
			
					switch (state_machine->rc_check)
					{
						case 1:
							break;

						case -1:
							state_machine->state->mav_state = MAV_STATE_CRITICAL;
							break;

						case -2:
							state_machine->state->mav_state = MAV_STATE_CRITICAL;
							break;
					}
				}
				break;

			case MAV_STATE_ACTIVE:
				switch(state_machine->channel_switches)
				{
					case 0:
						state_set_new_mode(state_machine->state,MAV_MODE_ATTITUDE_CONTROL);
						break;

					case 1:
						state_set_new_mode(state_machine->state,MAV_MODE_VELOCITY_CONTROL);
						break;

					case 2:
						state_set_new_mode(state_machine->state,MAV_MODE_POSITION_HOLD);
						break;

					case 3:
						state_set_new_mode(state_machine->state,MAV_MODE_GPS_NAVIGATION);
						break;
				}
		
				switch (state_machine->rc_check)
				{
					case 1:
						break;

					case -1:
						state_machine->state->mav_state = MAV_STATE_CRITICAL;
						break;

					case -2:
						state_machine->state->mav_state = MAV_STATE_CRITICAL;
						break;
				}
				break;

			case MAV_STATE_CRITICAL:
				switch(state_machine->channel_switches)
				{
					case 0:
						state_set_new_mode(state_machine->state,MAV_MODE_ATTITUDE_CONTROL);
						break;

					case 1:
						state_set_new_mode(state_machine->state,MAV_MODE_VELOCITY_CONTROL);
						break;

					case 2:
						state_set_new_mode(state_machine->state,MAV_MODE_POSITION_HOLD);
						break;

					case 3:
						state_set_new_mode(state_machine->state,MAV_MODE_GPS_NAVIGATION);
						break;
				}
			
				switch (state_machine->rc_check)
				{
					case 1:
						// !! only if receivers are back, switch into appropriate mode
						state_machine->state->mav_state = MAV_STATE_ACTIVE;
						state_machine->waypoint_handler->critical_behavior = CLIMB_TO_SAFE_ALT;
						state_machine->waypoint_handler->critical_next_state = false;
						break;

					case -1:
						if (state_test_if_in_mode(state_machine->state,MAV_MODE_ATTITUDE_CONTROL))
						{
							print_util_dbg_print("Attitude mode, direct to Emergency state_machine->state.\r");
							state_machine->state->mav_state = MAV_STATE_EMERGENCY;
						}
						break;

					case -2:
						if (state_test_if_in_mode(state_machine->state,MAV_MODE_ATTITUDE_CONTROL))
						{
							print_util_dbg_print("Attitude mode, direct to Emergency state_machine->state.\r");
							state_machine->state->mav_state = MAV_STATE_EMERGENCY;
						}
						if (state_machine->waypoint_handler->critical_landing)
						{
							state_machine->state->mav_state = MAV_STATE_EMERGENCY;
						}
					break;
				}
				break;

			case MAV_STATE_EMERGENCY:
				if (state_test_if_in_flag_mode(state_machine->state,MAV_MODE_FLAG_SAFETY_ARMED))
				{
					state_machine_switch_off_motors(state_machine);
					state_machine->state->mav_state = MAV_STATE_EMERGENCY;
				}

				switch (state_machine->rc_check)
				{
					case 1:
						state_machine->state->mav_state = MAV_STATE_STANDBY;
						break;

					case -1:
						break;

					case -2:
						break;
				}
				break;
		}
	
		if (state_machine->motor_state == -1)
		{
			state_machine_switch_off_motors(state_machine);
		}
	
	}
	
	state_machine->state->mav_mode_previous = state_machine->state->mav_mode;
	
	if (state_machine->state->simulation_mode_previous != state_machine->state->simulation_mode)
	{
		simulation_switch_between_reality_n_simulation(state_machine->sim_model);
		state_machine->state->simulation_mode_previous = state_machine->state->simulation_mode;
	}
	
	return TASK_RUN_SUCCESS;
}