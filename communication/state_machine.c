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
//#include "remote_controller.h"
#include "spektrum_satellite.h"
#include "led.h"
#include "print_util.h"
#include "state.h"

void state_machine_init(state_machine_t *state_machine, const state_machine_conf_t* state_machine_conf, state_t* state, mavlink_waypoint_handler_t* waypoint_handler, simulation_model_t *sim_model, remote_t* remote)
{
	state_machine->waypoint_handler = waypoint_handler;
	state_machine->state 			= state;
	state_machine->sim_model 		= sim_model;
	state_machine->remote 			= remote;
	state_machine->channel_switches = 0;
	state_machine->rc_check 		= 0;
	state_machine->motor_state 		= 0;

	state_machine->use_mode_from_remote = state_machine_conf->state_machine.use_mode_from_remote;
	
	print_util_dbg_print("State machine initialise.\r\n");
}


void state_machine_update(state_machine_t* state_machine)
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

	// Get new mode
	if ( (state_machine->use_mode_from_remote == 1)&&(rc_check != SIGNAL_LOST) )
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
				state_new = MAV_STATE_ACTIVE;
			
				// Tell other modules to reset position and re-compute waypoints
				state_machine->state->reset_position = true;
				state_machine->state->nav_plan_active = false;
			}
			break;
		
		case MAV_STATE_ACTIVE:
			if (state_machine->use_mode_from_remote == 1)
			{
				if ( rc_check != SIGNAL_GOOD )
				{
					state_new = MAV_STATE_CRITICAL;
				}
				else
				{
					if ( mode_new.ARMED == ARMED_OFF )
					{
						state_new = MAV_STATE_STANDBY;
						print_util_dbg_print("Switching off motors!\n");
					}
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
			
			// To get out of this state, if we are in the wrong use_mode_from_remote
			if (state_machine->use_mode_from_remote == 0)
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

}