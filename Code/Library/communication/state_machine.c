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
#include "spektrum_satellite.h"
#include "led.h"
#include "print_util.h"
#include "state.h"


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


void state_machine_init(state_machine_t *state_machine, state_t* state, mavlink_waypoint_handler_t* waypoint_handler, simulation_model_t *sim_model, remote_t* remote)
{
	state_machine->waypoint_handler = waypoint_handler;
	state_machine->state 			= state;
	state_machine->sim_model 		= sim_model;
	state_machine->remote 			= remote;
	state_machine->channel_switches = 0;
	state_machine->rc_check 		= 0;
	state_machine->motor_state 		= 0;

	state_machine->use_mode_from_remote = true;
	
	print_util_dbg_print("State machine initialise.\r\n");
}


void state_machine_rc_user_channels(state_machine_t* state_machine)
{
	state_machine->rc_check = remote_check(state_machine->remote);
	
	if (state_machine->rc_check == 1)
	{
		remote_controller_get_channel_mode(&state_machine->channel_switches);
	}
	
	if ((spektrum_satellite_get_neutral(RC_TRIM_P3) * RC_SCALEFACTOR) > 0.0f)
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


// task_return_t state_machine_set_mav_mode_n_state(state_machine_t* state_machine)
// {
	
// 	LED_Toggle(LED1);
	
// 	state_machine_rc_user_channels(state_machine);
	
// 	switch(state_machine->state->mav_state)
// 	{
// 		case MAV_STATE_UNINIT:
// 		case MAV_STATE_BOOT:
// 		case MAV_STATE_CALIBRATING:
// 		case MAV_STATE_POWEROFF:
// 		case MAV_STATE_ENUM_END:
// 			break;

// 		case MAV_STATE_STANDBY:
// 			if (state_machine->motor_state == 1)
// 			{
// 				switch(state_machine->channel_switches)
// 				{
// 					case 0:
// 						print_util_dbg_print("Switching on the motors!\n");

// 						state_machine->state->reset_position = true;
				
// 						state_machine->state->nav_plan_active = false;
// 						state_machine->state->mav_state = MAV_STATE_ACTIVE;
// 						state_set_new_mode(state_machine->state,MAV_MODE_ATTITUDE_CONTROL);
// 						break;

// 					default:
// 						print_util_dbg_print("Switches not ready, both should be pushed!\n");
// 						break;
// 				}
			
// 				switch (state_machine->rc_check)
// 				{
// 					case SIGNAL_GOOD:
// 						break;

// 					case SIGNAL_BAD:
// 						state_machine->state->mav_state = MAV_STATE_CRITICAL;
// 						break;

// 					case SIGNAL_LOST:
// 						state_machine->state->mav_state = MAV_STATE_CRITICAL;
// 						break;
// 				}
// 			}
// 			break;

// 		case MAV_STATE_ACTIVE:
// 			switch(state_machine->channel_switches)
// 			{
// 				case 0:
// 					state_set_new_mode(state_machine->state,MAV_MODE_ATTITUDE_CONTROL);
// 					break;

// 				case 1:
// 					state_set_new_mode(state_machine->state,MAV_MODE_VELOCITY_CONTROL);
// 					break;

// 				case 2:
// 					state_set_new_mode(state_machine->state,MAV_MODE_POSITION_HOLD);
// 					break;

// 				case 3:
// 					state_set_new_mode(state_machine->state,MAV_MODE_GPS_NAVIGATION);
// 					break;
// 			}
		
// 			switch (state_machine->rc_check)
// 			{
// 				case SIGNAL_GOOD:
// 					break;

// 				case SIGNAL_BAD:
// 					state_machine->state->mav_state = MAV_STATE_CRITICAL;
// 					break;

// 				case SIGNAL_LOST:
// 					state_machine->state->mav_state = MAV_STATE_CRITICAL;
// 					break;
// 			}
// 			break;

// 		case MAV_STATE_CRITICAL:
// 			switch(state_machine->channel_switches)
// 			{
// 				case 0:
// 					state_set_new_mode(state_machine->state,MAV_MODE_ATTITUDE_CONTROL);
// 					break;

// 				case 1:
// 					state_set_new_mode(state_machine->state,MAV_MODE_VELOCITY_CONTROL);
// 					break;

// 				case 2:
// 					state_set_new_mode(state_machine->state,MAV_MODE_POSITION_HOLD);
// 					break;

// 				case 3:
// 					state_set_new_mode(state_machine->state,MAV_MODE_GPS_NAVIGATION);
// 					break;
// 			}
			
// 			switch (state_machine->rc_check)
// 			{
// 				case SIGNAL_GOOD:
// 					// !! only if receivers are back, switch into appropriate mode
// 					state_machine->state->mav_state = MAV_STATE_ACTIVE;
// 					state_machine->waypoint_handler->critical_behavior = CLIMB_TO_SAFE_ALT;
// 					state_machine->waypoint_handler->critical_next_state = false;
// 					break;

// 				case SIGNAL_BAD:
// 					if (state_test_if_in_mode(state_machine->state,MAV_MODE_ATTITUDE_CONTROL))
// 					{
// 						print_util_dbg_print("Attitude mode, direct to Emergency state_machine->state.\r\n");
// 						state_machine->state->mav_state = MAV_STATE_EMERGENCY;
// 					}
// 					break;

// 				case SIGNAL_LOST:
// 					if (state_test_if_in_mode(state_machine->state,MAV_MODE_ATTITUDE_CONTROL))
// 					{
// 						print_util_dbg_print("Attitude mode, direct to Emergency state_machine->state.\r\n");
// 						state_machine->state->mav_state = MAV_STATE_EMERGENCY;
// 					}
// 					if (state_machine->waypoint_handler->critical_landing)
// 					{
// 						state_machine->state->mav_state = MAV_STATE_EMERGENCY;
// 					}
// 					break;
// 			}
// 			break;

// 		case MAV_STATE_EMERGENCY:
// 			if (state_test_if_in_flag_mode(state_machine->state,MAV_MODE_FLAG_SAFETY_ARMED))
// 			{
// 				state_machine_switch_off_motors(state_machine);
// 				state_machine->state->mav_state = MAV_STATE_EMERGENCY;
// 			}

// 			switch (state_machine->rc_check)
// 			{
// 				case SIGNAL_GOOD:
// 					state_machine->state->mav_state = MAV_STATE_STANDBY;
// 					break;

// 				case SIGNAL_BAD:
// 					break;

// 				case SIGNAL_LOST:
// 					break;
// 			}
// 			break;
// 	}
	
// 	if (state_machine->motor_state == -1)
// 	{
// 		state_machine_switch_off_motors(state_machine);
// 	}
	
// 	state_machine->state->mav_mode_previous = state_machine->state->mav_mode;
	
// 	if (state_machine->state->simulation_mode_previous != state_machine->state->simulation_mode)
// 	{
// 		//simulation_switch_between_reality_n_simulation(state_machine->sim_model);
// 		state_machine->state->simulation_mode_previous = state_machine->state->simulation_mode;
// 		state_machine->state->mav_mode.HIL = state_machine->state->simulation_mode;
// 	}
	
// 	return TASK_RUN_SUCCESS;
// }


void state_machine_update(state_machine_t* state_machine)
{
	mav_mode_t mode_current, mode_new;
	mav_state_t state_current, state_new;
	signal_quality_t rc_check;

	LED_Toggle(LED1);

	// Get current state
	state_current = state_machine->state->mav_state;

	// By default, set new state equal to current state
	state_new = state_current;

	// Get current mode
	mode_current = state_machine->state->mav_mode;

	// Get new mode
	if ( state_machine->use_mode_from_remote == true )
	{
		// Update mode from remote
		remote_mode_update(state_machine->remote);
		mode_new = remote_mode_get(state_machine->remote);
	}
	else
	{
		// By default, set new mode equal to current mode
		mode_new = mode_current;
	}
	
	// Get remote signal strength
	rc_check = remote_check(state_machine->remote);

	// Change state according to signal strength
	switch ( state_current )
	{
		case MAV_STATE_UNINIT:
		case MAV_STATE_BOOT:
		case MAV_STATE_CALIBRATING:
		case MAV_STATE_POWEROFF:
		case MAV_STATE_ENUM_END:
		break;

		case MAV_STATE_STANDBY:
		
			if ( mode_new.ARMED == ARMED_ON )
			{
				state_new = MAV_STATE_ACTIVE;
			
				// Tell other modules to reset position and re-compute waypoints
				state_machine->state->reset_position = true;
				state_machine->state->nav_plan_active = false;
			}
		break;
		
		case MAV_STATE_ACTIVE:
			if ( rc_check != SIGNAL_GOOD )
			{
				state_new = MAV_STATE_CRITICAL;
			}
			else
			{
				if ( mode_new.ARMED == ARMED_OFF )
				{
					state_new = MAV_STATE_STANDBY;
				}
			}
		break;

		case MAV_STATE_CRITICAL:			
			switch ( rc_check )
			{
				case SIGNAL_GOOD:
					state_new = MAV_STATE_ACTIVE;
				break;

				case SIGNAL_BAD:
					// Stay in critical mode
				break;

				case SIGNAL_LOST:
					// If in manual mode, do emergency landing
					if ( mode_current.MANUAL == MANUAL_ON )
					{
						state_new = MAV_STATE_EMERGENCY;
					}
					// If in another mode, stay in critical mode
					// higher level navigation module will take care of coming back home
				break;
			}
		break;
		
		case MAV_STATE_EMERGENCY:
			// Recovery is not possible -> switch off motors
			mode_new.ARMED = ARMED_OFF;
		break;
		
	}


	// Check simulation mode
	if ( state_machine->state->simulation_mode == true )
	{
		mode_new.HIL = HIL_ON;
	}
	else
	{
		mode_new.HIL = HIL_OFF;
	}


	// Check if we need to switch between simulation and reality	
	if ( mode_current.HIL != mode_new.HIL )
	{
		if ( mode_new.HIL == HIL_ON )
		{
			// reality -> simulation
			simulation_switch_from_reality_to_simulation( state_machine->sim_model );
		}
		else
		{
			// simulation -> reality
			simulation_switch_from_simulation_to_reality( state_machine->sim_model );

			// For safety, switch off the motors
			state_machine_switch_off_motors(state_machine);
		}
	}

	// Finally, write new modes and states
	state_machine->state->mav_mode = mode_new;
	state_machine->state->mav_state = state_new;

}