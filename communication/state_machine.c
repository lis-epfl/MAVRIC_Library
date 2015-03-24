/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*******************************************************************************
 * \file state_machine.c
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 * \author Julien Lecoeur
 *   
 * \brief Handles transitions between states and modes
 *
 ******************************************************************************/


#include "state_machine.h"
#include "spektrum_satellite.h"
#include "led.h"
#include "print_util.h"
#include "state.h"
#include "time_keeper.h"
#include "battery.h"

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief	Returns the value of the mode from the desired source input
 *
 * \param	state_machine			The pointer to the state_machine structure
 * \param	mode_current			The current mode of the MAV
 * \param	rc_check				The current status of the remote controller
 *
 * \return	The value of the mode
 */
mav_mode_t state_machine_get_mode_from_source(state_machine_t* state_machine, mav_mode_t mode_current, signal_quality_t rc_check );


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

mav_mode_t state_machine_get_mode_from_source(state_machine_t* state_machine, mav_mode_t mode_current, signal_quality_t rc_check )
{
	mav_mode_t new_mode = mode_current;
	
	switch (state_machine->state->source_mode)
	{
		case GND_STATION:
			new_mode = mode_current;
			// The ARMED flag of the remote is set to the desired flag (avoid sudden cut
			// off if the remote is reactivated
			state_machine->remote->mode.current_desired_mode.ARMED = mode_current.ARMED;
			
			break;
		case REMOTE:
			if(rc_check != SIGNAL_LOST)
			{
				// Update mode from remote
				remote_mode_update(state_machine->remote);
				new_mode = remote_mode_get(state_machine->remote);
			}
			break;
		case JOYSTICK:
			new_mode = joystick_parsing_get_mode(state_machine->joystick);
			// The ARMED flag of the remote is set to the desired flag (avoid sudden cut
			// off if the remote is reactivated
			state_machine->remote->mode.current_desired_mode.ARMED = mode_current.ARMED;
			break;
		default:
			new_mode = mode_current;
			break;
	}
	
	return new_mode;
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool state_machine_init(	state_machine_t *state_machine,
							state_t* state, 
							simulation_model_t *sim_model, 
							remote_t* remote,
							joystick_parsing_t* joystick)
{
	bool init_success = true;
	
	state_machine->state 			= state;
	state_machine->sim_model 		= sim_model;
	state_machine->remote 			= remote;
	state_machine->joystick = joystick;
	
	state_machine->channel_switches = 0;
	state_machine->rc_check 		= 0;
	state_machine->motor_state 		= 0;
	
	state_machine->low_battery_counter	= 0;
	state_machine->low_battery_update	= 0;
	
	print_util_dbg_print("[STATE MACHINE] Initialised.\r\n");
	
	return init_success;
}


task_return_t state_machine_update(state_machine_t* state_machine)
{
	mav_mode_t mode_current, mode_new;
	mav_state_t state_current, state_new;
	signal_quality_t rc_check;

	// Get current state
	state_current = state_machine->state->mav_state;

	// By default, set new state equal to current state
	state_new = state_current;

	// Get current mode
	mode_current = state_machine->state->mav_mode;

	// Get remote signal strength
	if (state_machine->state->remote_active == 1)
	{
		rc_check = remote_check(state_machine->remote);
	}
	else
	{
		rc_check = SIGNAL_GOOD;
	}

	mode_new = state_machine_get_mode_from_source(state_machine, mode_current, rc_check);

	battery_update(&state_machine->state->battery,state_machine->state->analog_monitor->avg[ANALOG_RAIL_10]);

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
			state_machine->state->in_the_air = false;
			
			if ( mode_new.ARMED == ARMED_ON )
			{
				print_util_dbg_print("Switching from state_machine.\r\n");
				state_switch_to_active_mode(state_machine->state, &state_new);
			}
			break;
		
		case MAV_STATE_ACTIVE:
			if ((state_machine->state->source_mode == REMOTE)||(state_machine->state->source_mode == JOYSTICK))
			{
				if ( (state_machine->state->source_mode == REMOTE)&&(rc_check != SIGNAL_GOOD) )
				{
					state_new = MAV_STATE_CRITICAL;
				}
				else
				{
					if ( mode_new.ARMED == ARMED_OFF )
					{
						state_new = MAV_STATE_STANDBY;
						print_util_dbg_print("Switching off motors!\r\n");
					}
				}
			}
			//check battery level
			if( state_machine->state->battery.is_low )
			{
				print_util_dbg_print("Battery low! Performing critical landing.\r\n");
				state_new = MAV_STATE_CRITICAL;
			}
			break;

		case MAV_STATE_CRITICAL:			
			switch ( rc_check )
			{
				case SIGNAL_GOOD:
					if( !state_machine->state->battery.is_low)
					{
						state_new = MAV_STATE_ACTIVE;
					}
					break;

				case SIGNAL_BAD:
					// Stay in critical mode
					break;

				case SIGNAL_LOST:
					// If in manual mode, do emergency landing (cut off motors)
					if ( (mode_current.MANUAL == MANUAL_ON) && (mode_current.STABILISE == STABILISE_OFF) )
					{
						print_util_dbg_print("Switch to Emergency mode!\r\n");
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
			state_machine->remote->mode.current_desired_mode.ARMED = ARMED_OFF;
			
			if( !state_machine->state->battery.is_low)
			{
				// To get out of this state, if we are in the wrong use_mode_from_remote
				if (state_machine->state->source_mode != REMOTE)
				{
					state_new = MAV_STATE_STANDBY;
				}
				
				switch ( rc_check )
				{
					case SIGNAL_GOOD:
						state_new = MAV_STATE_STANDBY;
						break;

					case SIGNAL_BAD:
						// Stay in emergency mode
						break;

					case SIGNAL_LOST:
						// Stay in emergency mode
						break;
				}
			}
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
			
			state_new = MAV_STATE_STANDBY;
			mode_new.byte = MAV_MODE_MANUAL_DISARMED;
			mode_new.HIL = HIL_ON;
		}
		else
		{
			// simulation -> reality
			simulation_switch_from_simulation_to_reality( state_machine->sim_model );

			state_new = MAV_STATE_STANDBY;
			mode_new.byte = MAV_MODE_SAFE;

			// For safety, switch off the motors
			print_util_dbg_print("Switching off motors!\n");
		}
	}
	

	// Finally, write new modes and states
	state_machine->state->mav_mode = mode_new;
	state_machine->state->mav_state = state_new;

	return TASK_RUN_SUCCESS;
}