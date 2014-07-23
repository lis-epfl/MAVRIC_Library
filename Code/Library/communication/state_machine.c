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

/**
 * \brief            	This function does bullshit
 * \details  			1) Switch on/off collision avoidance
 * 						2) Switch on/off the motor
 * 						3) Check the receivers
 * 
 * \param	chanSwitch	The pointer to set the switch mode
 * \param	rc_check	The pointer to the state of the remote
 * \param	motorstate	The pointer to the motor state
 */
void tasks_rc_user_channels(uint8_t *chanSwitch, int8_t *rc_check, int8_t *motor_state);

void state_machine_init(state_machine_t *state_machine, state_structure_t* state_structure, mavlink_waypoint_handler_t* waypoint_handler, navigation_t* navigation_data)
{
	state_machine->state_structure = state_structure;
	state_machine->waypoint_handler = waypoint_handler;
	state_machine->navigation_data = navigation_data;
}

void state_machine_rc_user_channels(uint8_t *chanSwitch, int8_t *rc_check, int8_t *motor_state)
{
	
	remote_controller_get_channel_mode(chanSwitch);
	
	if ((remote_dsm2_rc_get_channel_neutral(RC_TRIM_P3) * RC_SCALEFACTOR) > 0.0f)
	{
		state_machine->navigationData.collision_avoidance = true;
	}
	else
	{
		state_machine->navigationData.collision_avoidance = false;
	}
	
	remote_controller_get_motor_state(motor_state);
	
	*rc_check = remote_dsm2_rc_check_receivers();
}

task_return_t state_machine_set_mav_mode_n_state(void* arg)
{
	uint8_t channelSwitches = 0;
	int8_t RC_check = 0;
	int8_t motor_switch = 0;
	
	float distFromHomeSqr;
	
	LED_Toggle(LED1);
	
	tasks_rc_user_channels(&channelSwitches,&RC_check, &motor_switch);
	
	switch(state_machine->state_structure.mav_state)
	{
		case MAV_STATE_BOOT:
		break;

		case MAV_STATE_CALIBRATING:
		break;

		case MAV_STATE_STANDBY:
			if (motor_switch == 1)
			{
				switch(channelSwitches)
				{
					case 0:
						print_util_dbg_print("Switching on the motors!\n");

						position_estimation_reset_home_altitude(&state_machine->position_estimator);
				
						state_machine->waypoint_handler.waypoint_set = false;
						state_set_new_mode(&state_machine->state_structure,MAV_MODE_ATTITUDE_CONTROL);
						break;

					default:
						print_util_dbg_print("Switches not ready, both should be pushed!\n");
						break;
				}
			}
			if (state_test_if_in_flag_mode(&state_machine->state_structure,MAV_MODE_FLAG_SAFETY_ARMED))
			{
				switch (channelSwitches)
				{
					case 0:
						state_set_new_mode(&state_machine->state_structure,MAV_MODE_ATTITUDE_CONTROL);
						if (state_machine->waypoint_handler.in_the_air)
						{
							state_machine->state_structure.mav_state = MAV_STATE_ACTIVE;
						}
						break;

					case 1:
						state_set_new_mode(&state_machine->state_structure,MAV_MODE_VELOCITY_CONTROL);
						if (state_machine->waypoint_handler.in_the_air)
						{
							state_machine->state_structure.mav_state = MAV_STATE_ACTIVE;
						}
						break;

					case 2:
						if (state_machine->waypoint_handler.in_the_air)
						{
							state_set_new_mode(&state_machine->state_structure,MAV_MODE_POSITION_HOLD);
					
							// Activate automatic take-off mode
							if (state_test_if_first_time_in_mode(&state_machine->state_structure,MAV_MODE_POSITION_HOLD))
							{
								waypoint_handler_waypoint_take_off(&state_machine->waypoint_handler);
							}
					
							distFromHomeSqr =	SQR(state_machine->position_estimator.localPosition.pos[X] - state_machine->waypoint_handler.waypoint_hold_coordinates.pos[X]) +
							SQR(state_machine->position_estimator.localPosition.pos[Y] - state_machine->waypoint_handler.waypoint_hold_coordinates.pos[Y]) +
							SQR(state_machine->position_estimator.localPosition.pos[Z] - state_machine->waypoint_handler.waypoint_hold_coordinates.pos[Z]);
					
							if (state_machine->waypoint_handler.dist2wp_sqr <= 16.0f)
							{
								state_machine->state_structure.mav_state = MAV_STATE_ACTIVE;
								print_util_dbg_print("Automatic take-off finised, distFromHomeSqr (10x):");
								print_util_dbg_print_num(distFromHomeSqr * 10.0f,10);
								print_util_dbg_print(".\n");
							}
						}
						break;

					case 3:
						if (state_machine->waypoint_handler.in_the_air)
						{
							//state_machine->state_structure.mav_state = MAV_STATE_ACTIVE;
							state_set_new_mode(&state_machine->state_structure,MAV_MODE_GPS_NAVIGATION);
					
							// Automatic take-off mode
							if(state_test_if_first_time_in_mode(&state_machine->state_structure,MAV_MODE_GPS_NAVIGATION))
							{
								waypoint_handler_waypoint_take_off(&state_machine->waypoint_handler);
							}

							if (!state_machine->waypoint_handler.waypoint_set)
							{
								waypoint_handler_waypoint_init(&state_machine->waypoint_handler);
							}

							distFromHomeSqr =	SQR(state_machine->position_estimator.localPosition.pos[X] - state_machine->waypoint_handler.waypoint_hold_coordinates.pos[X]) +
							SQR(state_machine->position_estimator.localPosition.pos[Y] - state_machine->waypoint_handler.waypoint_hold_coordinates.pos[Y]) +
							SQR(state_machine->position_estimator.localPosition.pos[Z] - state_machine->waypoint_handler.waypoint_hold_coordinates.pos[Z]);
					
							if (state_machine->waypoint_handler.dist2wp_sqr <= 16.0f)
							{
								state_machine->state_structure.mav_state = MAV_STATE_ACTIVE;
								print_util_dbg_print("Automatic take-off finised, distFromHomeSqr (10x):");
								print_util_dbg_print_num(distFromHomeSqr * 10.0f,10);
								print_util_dbg_print(".\n");
							}
						}
						break;
				}
			
				switch (RC_check)
				{
					case 1:
						break;

					case -1:
						state_machine->state_structure.mav_state = MAV_STATE_CRITICAL;
						break;

					case -2:
						state_machine->state_structure.mav_state = MAV_STATE_CRITICAL;
						break;
				}
				if (remote_controller_get_thrust_from_remote() > -0.7f)
				{
					state_machine->waypoint_handler.in_the_air = true;
				}
			}

			if (motor_switch == -1)
			{
				switch_off_motors();
			}
		
			break;

		case MAV_STATE_ACTIVE:
			switch(channelSwitches)
			{
				case 0:
				state_set_new_mode(&state_machine->state_structure,MAV_MODE_ATTITUDE_CONTROL);
				break;

				case 1:
				state_set_new_mode(&state_machine->state_structure,MAV_MODE_VELOCITY_CONTROL);
				break;

				case 2:
				state_set_new_mode(&state_machine->state_structure,MAV_MODE_POSITION_HOLD);
				if (state_test_if_first_time_in_mode(&state_machine->state_structure,MAV_MODE_POSITION_HOLD))
				{
					waypoint_handler_waypoint_hold_init(&state_machine->waypoint_handler,state_machine->position_estimator.localPosition);
				}
				break;

				case 3:
				state_set_new_mode(&state_machine->state_structure,MAV_MODE_GPS_NAVIGATION);
				if (state_test_if_first_time_in_mode(&state_machine->state_structure,MAV_MODE_GPS_NAVIGATION))
				{
					state_machine->waypoint_handler.auto_landing_behavior = DESCENT_TO_SMALL_ALTITUDE;
					waypoint_handler_waypoint_hold_init(&state_machine->waypoint_handler,state_machine->position_estimator.localPosition);
				}
				break;
			}
		
			if (state_test_if_in_flag_mode(&state_machine->state_structure,MAV_MODE_FLAG_AUTO_ENABLED))
			{
				if (!state_machine->waypoint_handler.waypoint_set)
				{
					waypoint_handler_waypoint_init(&state_machine->waypoint_handler);
				}

				waypoint_handler_waypoint_navigation_handler(&state_machine->waypoint_handler);
			}
		
			if (motor_switch == -1)
			{
				switch_off_motors();
			}
		
			switch (RC_check)
			{
				case 1:
				break;

				case -1:
				state_machine->state_structure.mav_state = MAV_STATE_CRITICAL;
				break;

				case -2:
				state_machine->state_structure.mav_state = MAV_STATE_CRITICAL;
				break;
			}
			break;

		case MAV_STATE_CRITICAL:
			switch(channelSwitches)
			{
				case 0:
					state_set_new_mode(&state_machine->state_structure,MAV_MODE_ATTITUDE_CONTROL);
					break;

				case 1:
					state_set_new_mode(&state_machine->state_structure,MAV_MODE_VELOCITY_CONTROL);
					break;

				case 2:
					state_set_new_mode(&state_machine->state_structure,MAV_MODE_POSITION_HOLD);
					break;

				case 3:
					state_set_new_mode(&state_machine->state_structure,MAV_MODE_GPS_NAVIGATION);
					break;
			}

			if (motor_switch == -1)
			{
				switch_off_motors();
			}
		
			// In MAV_MODE_VELOCITY_CONTROL, MAV_MODE_POSITION_HOLD and MAV_MODE_GPS_NAVIGATION
			if (state_test_if_in_flag_mode(&state_machine->state_structure,MAV_MODE_FLAG_STABILIZE_ENABLED))
			{
				waypoint_handler_waypoint_critical_handler(&state_machine->waypoint_handler);
			}
		
			switch (RC_check)
			{
				case 1:
					// !! only if receivers are back, switch into appropriate mode
					state_machine->state_structure.mav_state = MAV_STATE_ACTIVE;
					state_machine->waypoint_handler.critical_behavior = CLIMB_TO_SAFE_ALT;
					state_machine->waypoint_handler.critical_next_state = false;
					break;

				case -1:
					break;

				case -2:
					if (state_machine->waypoint_handler.critical_landing)
					{
						state_machine->state_structure.mav_state = MAV_STATE_EMERGENCY;
					}

				break;
			}
			break;

		case MAV_STATE_EMERGENCY:
			state_set_new_mode(&state_machine->state_structure,MAV_MODE_SAFE);
			switch (RC_check)
			{
				case 1:
				state_machine->state_structure.mav_state = MAV_STATE_STANDBY;
				break;

				case -1:
				break;

				case -2:
				break;
			}
			break;
	}
	
	state_machine->state_structure.mav_mode_previous = state_machine->state_structure.mav_mode;
	
	if (state_machine->state_structure.simulation_mode_previous != state_machine->state_structure.simulation_mode)
	{
		simulation_switch_between_reality_n_simulation(&state_machine->sim_model);
		state_machine->state_structure.simulation_mode_previous = state_machine->state_structure.simulation_mode;
	}
	
	return TASK_RUN_SUCCESS;
}