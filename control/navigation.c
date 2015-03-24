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
 * \file navigation.c
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *   
 * \brief Waypoint navigation controller
 *
 ******************************************************************************/


#include "navigation.h"
#include "print_util.h"
#include "time_keeper.h"
#include "constants.h"

#define KP_YAW 0.2f ///< Yaw gain for the navigation controller 

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief	Sets the automatic takeoff waypoint
 *
 * \param	waypoint_handler		The pointer to the waypoint handler structure
 */
static void navigation_waypoint_take_off_init(mavlink_waypoint_handler_t* waypoint_handler);

/**
 * \brief					Computes the relative position and distance to the given way point
 *
 * \param	waypoint_pos		Local coordinates of the waypoint
 * \param	rel_pos			Array to store the relative 3D position of the waypoint
 * \param	local_pos		The 3D array of the actual position
 *
 * \return					Distance to waypoint squared
 */
static float navigation_set_rel_pos_n_dist2wp(float waypoint_pos[], float rel_pos[], const float local_pos[3]);

/**
 * \brief					Sets the Robot speed to reach waypoint
 *
 * \param	rel_pos			Relative position of the waypoint
 * \param	navigation	The structure of navigation data
 */
static void navigation_set_speed_command(float rel_pos[], navigation_t* navigation);

/**
 * \brief						Navigates the robot towards waypoint waypoint_input in 3D velocity command mode
 *
 * \param	waypoint_input		Destination waypoint in local coordinate system
 * \param	navigation			The navigation structure
 */
static void navigation_run(local_coordinates_t waypoint_input, navigation_t* navigation);

/**
 * \brief	Sets auto-takeoff procedure from a MAVLink command message MAV_CMD_NAV_TAKEOFF
 *
 * \param	navigation			The pointer to the navigation structure
 * \param	packet				The pointer to the structure of the MAVLink command message long
 * 
 * \return	The MAV_RESULT of the command
 */
static mav_result_t navigation_set_auto_takeoff(navigation_t *navigation, mavlink_command_long_t* packet);

/**
 * \brief	Drives the auto landing procedure from the MAV_CMD_NAV_LAND message long
 *
 * \param	navigation			The pointer to the navigation structure
 * \param	packet					The pointer to the structure of the MAVLink command message long
 * 
 * \return	The MAV_RESULT of the command
 */
static mav_result_t navigation_set_auto_landing(navigation_t* navigation, mavlink_command_long_t* packet);

/**
 * \brief	Check if the nav mode is equal to the state mav mode
 *
 * \param	navigation			The pointer to the navigation structure
 *
 * \return	True if the flag STABILISE, GUIDED and ARMED are equal, false otherwise
 */
static bool navigation_mode_change(navigation_t* navigation);

/**
 * \brief	Drives the automatic takeoff procedure
 *
 * \param	navigation		The pointer to the navigation structure in central_data
 */
static void navigation_waypoint_take_off_handler(navigation_t* navigation);

/**
 * \brief	Drives the hold position procedure
 *
 * \param	navigation		The pointer to the navigation structure in central_data
 */
static void navigation_hold_position_handler(navigation_t* navigation);

/**
 * \brief	Drives the GPS navigation procedure
 *
 * \param	navigation		The pointer to the navigation structure in central_data
 */
static void navigation_waypoint_navigation_handler(navigation_t* navigation);

/**
 * \brief	Drives the critical navigation behavior
 *
 * \param	navigation		The pointer to the navigation structure in central_data
 */
static void navigation_critical_handler(navigation_t* navigation);

/**
 * \brief	Drives the auto-landing navigation behavior
 *
 * \param	navigation		The pointer to the navigation structure in central_data
 */
static void navigation_auto_landing_handler(navigation_t* navigation);

/**
 * \brief	Start/Stop the navigation
 *
 * \param	navigation			The pointer to the navigation structure
 * \param	packet					The pointer to the structure of the MAVLink command message long
 * 
 * \return	The MAV_RESULT of the command
 */
static mav_result_t navigation_start_stop_navigation(navigation_t* navigation, mavlink_command_long_t* packet);

/**
 * \brief	Drives the stopping behavior
 *
 * \param	navigation		The pointer to the navigation structure in central_data
 */
static void navigation_stopping_handler(navigation_t* navigation);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static void navigation_waypoint_take_off_init(mavlink_waypoint_handler_t* waypoint_handler)
{
	print_util_dbg_print("Automatic take-off, will hold position at: (");
	print_util_dbg_print_num(waypoint_handler->position_estimation->local_position.pos[X],10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(waypoint_handler->position_estimation->local_position.pos[Y],10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(-10.0f,10);
	print_util_dbg_print("), with heading of: ");
	print_util_dbg_print_num((int32_t)(waypoint_handler->position_estimation->local_position.heading*180.0f/3.14f),10);
	print_util_dbg_print("\r\n");

	waypoint_handler->waypoint_hold_coordinates = waypoint_handler->position_estimation->local_position;
	waypoint_handler->waypoint_hold_coordinates.pos[Z] = -10.0f;
	
	aero_attitude_t aero_attitude;
	aero_attitude=coord_conventions_quat_to_aero(waypoint_handler->ahrs->qe);
	waypoint_handler->waypoint_hold_coordinates.heading = aero_attitude.rpy[2];
	
	waypoint_handler->dist2wp_sqr = 100.0f; // same position, 10m above => dist_sqr = 100.0f
	
	waypoint_handler->hold_waypoint_set = true;
}

static float navigation_set_rel_pos_n_dist2wp(float waypoint_pos[], float rel_pos[], const float local_pos[3])
{
	float dist2wp_sqr;
	
	rel_pos[X] = (float)(waypoint_pos[X] - local_pos[X]);
	rel_pos[Y] = (float)(waypoint_pos[Y] - local_pos[Y]);
	rel_pos[Z] = (float)(waypoint_pos[Z] - local_pos[Z]);
	
	dist2wp_sqr = vectors_norm_sqr(rel_pos);
	
	return dist2wp_sqr;
}


static void navigation_set_speed_command(float rel_pos[], navigation_t* navigation)
{
	float  norm_rel_dist;
	float v_desired = 0.0f;
	quat_t qtmp1, qtmp2;
	
	float dir_desired_bf[3];
	float rel_heading;
	
	mav_mode_t mode = navigation->state->mav_mode;
	
	norm_rel_dist = sqrt(navigation->waypoint_handler->dist2wp_sqr);
	
	if (norm_rel_dist < 0.0005f)
	{
		norm_rel_dist += 0.0005f;
	}
	
	// calculate dir_desired in local frame
	// vel = qe-1 * rel_pos * qe
	qtmp1 = quaternions_create_from_vector(rel_pos);
	qtmp2 = quaternions_global_to_local(*navigation->qe,qtmp1);
	dir_desired_bf[0] = qtmp2.v[0]; dir_desired_bf[1] = qtmp2.v[1]; dir_desired_bf[2] = qtmp2.v[2];
	
	dir_desired_bf[2] = rel_pos[2];
	
	norm_rel_dist = max(navigation->cruise_speed,norm_rel_dist);
	if ((mode.AUTO == AUTO_ON) && ((navigation->state->nav_plan_active&&(!navigation->stop_nav)&&(!navigation->auto_takeoff)&&(!navigation->auto_landing))||((navigation->state->mav_state == MAV_STATE_CRITICAL)&&(navigation->critical_behavior == FLY_TO_HOME_WP))))
	{
		
		if( ((maths_f_abs(rel_pos[X])<=1.0f)&&(maths_f_abs(rel_pos[Y])<=1.0f)) || ((maths_f_abs(rel_pos[X])<=5.0f)&&(maths_f_abs(rel_pos[Y])<=5.0f)&&(maths_f_abs(rel_pos[Z])>=3.0f)) )
		{
			rel_heading = 0.0f;
		}
		else
		{
		rel_heading = maths_calc_smaller_angle(atan2(rel_pos[Y],rel_pos[X]) - navigation->position_estimation->local_position.heading);
		}
		
		navigation->wpt_nav_controller.clip_max = navigation->cruise_speed;
		v_desired = pid_controller_update_dt(&navigation->wpt_nav_controller, norm_rel_dist, navigation->dt);
	}
	else
	{
		rel_heading = 0.0f;
		navigation->hovering_controller.clip_max = navigation->cruise_speed;
		v_desired = pid_controller_update_dt(&navigation->hovering_controller, norm_rel_dist, navigation->dt);
	}
	
	if (v_desired *  maths_f_abs(dir_desired_bf[Z]) > navigation->max_climb_rate * norm_rel_dist ) 
	{
		v_desired = navigation->max_climb_rate * norm_rel_dist /maths_f_abs(dir_desired_bf[Z]);
	}
	
	dir_desired_bf[X] = v_desired * dir_desired_bf[X] / norm_rel_dist;
	dir_desired_bf[Y] = v_desired * dir_desired_bf[Y] / norm_rel_dist;
	dir_desired_bf[Z] = v_desired * dir_desired_bf[Z] / norm_rel_dist;
	
	/*
	loop_count = loop_count++ %50;
	if (loop_count == 0)
	{
		mavlink_msg_named_value_float_send(MAVLINK_COMM_0,time_keeper_get_millis(),"v_desired",v_desired*100);
		mavlink_msg_named_value_float_send(MAVLINK_COMM_0,time_keeper_get_millis(),"act_vel",vector_norm(navigation->position_estimation->vel_bf)*100);
		print_util_dbg_print("Desired_vel_Bf(x100): (");
		print_util_dbg_print_num(dir_desired_bf[X] * 100,10);
		print_util_dbg_print_num(dir_desired_bf[Y] * 100,10);
		print_util_dbg_print_num(dir_desired_bf[Z] * 100,10);
		print_util_dbg_print("). \n");
		print_util_dbg_print("Actual_vel_bf(x100): (");
		print_util_dbg_print_num(navigation->position_estimation->vel_bf[X] * 100,10);
		print_util_dbg_print_num(navigation->position_estimation->vel_bf[Y] * 100,10);
		print_util_dbg_print_num(navigation->position_estimation->vel_bf[Z] * 100,10);
		print_util_dbg_print("). \n");
		print_util_dbg_print("Actual_pos(x100): (");
		print_util_dbg_print_num(navigation->position_estimation->local_position.pos[X] * 100,10);
		print_util_dbg_print_num(navigation->position_estimation->local_position.pos[Y] * 100,10);
		print_util_dbg_print_num(navigation->position_estimation->local_position.pos[Z] * 100,10);
		print_util_dbg_print("). \n");
	}
	*/

	navigation->controls_nav->tvel[X] = dir_desired_bf[X];
	navigation->controls_nav->tvel[Y] = dir_desired_bf[Y];
	navigation->controls_nav->tvel[Z] = dir_desired_bf[Z];		
	navigation->controls_nav->rpy[YAW] = KP_YAW * rel_heading;
}

static void navigation_run(local_coordinates_t waypoint_input, navigation_t* navigation)
{
	float rel_pos[3];
	
	// Control in translational speed of the platform
	navigation->waypoint_handler->dist2wp_sqr = navigation_set_rel_pos_n_dist2wp(waypoint_input.pos,
																					rel_pos,
																					navigation->position_estimation->local_position.pos);
	navigation_set_speed_command(rel_pos, navigation);
	
	navigation->controls_nav->theading=waypoint_input.heading;
}

static mav_result_t navigation_set_auto_takeoff(navigation_t *navigation, mavlink_command_long_t* packet)
{	
	mav_result_t result;
	
	if (!navigation->state->in_the_air)
	{
		print_util_dbg_print("Starting automatic take-off from button\n");
		navigation->auto_takeoff = true;

		result = MAV_RESULT_ACCEPTED;
	}
	else
	{
		result = MAV_RESULT_DENIED;
	}
	
	return result;
}

static mav_result_t navigation_set_auto_landing(navigation_t* navigation, mavlink_command_long_t* packet)
{
	mav_result_t result;

	if (navigation->state->in_the_air)
	{
		result = MAV_RESULT_ACCEPTED;
		if (!navigation->auto_landing)
		{
			navigation->auto_landing_behavior = DESCENT_TO_SMALL_ALTITUDE;
		}
		
		navigation->auto_landing = true;
		navigation->auto_landing_next_state = false;
		print_util_dbg_print("Auto-landing procedure initialised.\r\n");
	}
	else
	{
		result = MAV_RESULT_DENIED;
	}

	return result;
}

static bool navigation_mode_change(navigation_t* navigation)
{
	mav_mode_t mode_local = navigation->state->mav_mode;
	mav_mode_t mode_nav = navigation->mode;
	
	bool result = false;
	
	if ((mode_local.STABILISE == mode_nav.STABILISE)&&(mode_local.GUIDED == mode_nav.GUIDED)&&(mode_local.AUTO == mode_nav.AUTO))
	{
		result = true;
	}
	
	return result;
}

static void navigation_waypoint_take_off_handler(navigation_t* navigation)
{
	if (!navigation->waypoint_handler->hold_waypoint_set)
	{
		navigation_waypoint_take_off_init(navigation->waypoint_handler);
	}
	if (!navigation->state->nav_plan_active)
	{
		waypoint_handler_nav_plan_init(navigation->waypoint_handler);
	}
	
	//if (navigation->mode == navigation->state->mav_mode.byte)
	if (navigation_mode_change(navigation))
	{
		if (navigation->waypoint_handler->dist2wp_sqr <= 16.0f)
		{
			//state_machine->state->mav_state = MAV_STATE_ACTIVE;
			navigation->state->in_the_air = true;
			navigation->auto_takeoff = false;
			print_util_dbg_print("Automatic take-off finised, dist2wp_sqr (10x):");
			print_util_dbg_print_num(navigation->waypoint_handler->dist2wp_sqr * 10.0f,10);
			print_util_dbg_print(".\r\n");
		}
	}
}


static void navigation_hold_position_handler(navigation_t* navigation)
{
	//if (navigation->mode != navigation->state->mav_mode.byte)
	if (!navigation_mode_change(navigation))
	{
		navigation->waypoint_handler->hold_waypoint_set = false;
	}
	
	if (!navigation->state->nav_plan_active)
	{
		waypoint_handler_nav_plan_init(navigation->waypoint_handler);
	}
	
	if (!navigation->waypoint_handler->hold_waypoint_set)
	{
		navigation_waypoint_hold_init(navigation->waypoint_handler, navigation->waypoint_handler->position_estimation->local_position);
	}
}

static void navigation_waypoint_navigation_handler(navigation_t* navigation)
{
	//if (navigation->mode != navigation->state->mav_mode.byte)
	if (!navigation_mode_change(navigation))
	{
		navigation->waypoint_handler->hold_waypoint_set = false;
	}
	
	if (navigation->state->nav_plan_active)
	{
		float rel_pos[3];
		
		for (uint8_t i=0;i<3;i++)
		{
			rel_pos[i] = navigation->waypoint_handler->waypoint_coordinates.pos[i]-navigation->waypoint_handler->position_estimation->local_position.pos[i];
		}
		navigation->waypoint_handler->dist2wp_sqr = vectors_norm_sqr(rel_pos);
		
		if (navigation->waypoint_handler->dist2wp_sqr < (navigation->waypoint_handler->current_waypoint.param2*navigation->waypoint_handler->current_waypoint.param2))
		{
			print_util_dbg_print("Waypoint Nr");
			print_util_dbg_print_num(navigation->waypoint_handler->current_waypoint_count,10);
			print_util_dbg_print(" reached, distance:");
			print_util_dbg_print_num(sqrt(navigation->waypoint_handler->dist2wp_sqr),10);
			print_util_dbg_print(" less than :");
			print_util_dbg_print_num(navigation->waypoint_handler->current_waypoint.param2,10);
			print_util_dbg_print(".\r\n");
			
			mavlink_message_t msg;
			mavlink_msg_mission_item_reached_pack( 	navigation->mavlink_stream->sysid,
													navigation->mavlink_stream->compid,
													&msg,
													navigation->waypoint_handler->current_waypoint_count);
			mavlink_stream_send(navigation->mavlink_stream, &msg);
			
			navigation->waypoint_handler->travel_time = time_keeper_get_millis() - navigation->waypoint_handler->start_wpt_time;
			
			navigation->waypoint_handler->waypoint_list[navigation->waypoint_handler->current_waypoint_count].current = 0;
			if((navigation->waypoint_handler->current_waypoint.autocontinue == 1)&&(navigation->waypoint_handler->number_of_waypoints>1))
			{
				print_util_dbg_print("Autocontinue towards waypoint Nr");
				
				navigation->waypoint_handler->start_wpt_time = time_keeper_get_millis();
				
				if (navigation->waypoint_handler->current_waypoint_count == (navigation->waypoint_handler->number_of_waypoints-1))
				{
					navigation->waypoint_handler->current_waypoint_count = 0;
				}
				else
				{
					navigation->waypoint_handler->current_waypoint_count++;
				}
				print_util_dbg_print_num(navigation->waypoint_handler->current_waypoint_count,10);
				print_util_dbg_print("\r\n");
				navigation->waypoint_handler->waypoint_list[navigation->waypoint_handler->current_waypoint_count].current = 1;
				navigation->waypoint_handler->current_waypoint = navigation->waypoint_handler->waypoint_list[navigation->waypoint_handler->current_waypoint_count];
				navigation->waypoint_handler->waypoint_coordinates = waypoint_handler_set_waypoint_from_frame(&navigation->waypoint_handler->current_waypoint, navigation->waypoint_handler->position_estimation->local_position.origin);
				
				mavlink_message_t msg;
				mavlink_msg_mission_current_pack( 	navigation->mavlink_stream->sysid,
													navigation->mavlink_stream->compid,
													&msg,
													navigation->waypoint_handler->current_waypoint_count);
				mavlink_stream_send(navigation->mavlink_stream, &msg);
				
			}
			else
			{
				navigation->state->nav_plan_active = false;
				print_util_dbg_print("Stop\r\n");
				
				navigation_waypoint_hold_init(navigation->waypoint_handler, navigation->waypoint_handler->waypoint_coordinates);
			}
		}
	}
	else
	{
		if (!navigation->waypoint_handler->hold_waypoint_set)
		{
			navigation_waypoint_hold_init(navigation->waypoint_handler, navigation->waypoint_handler->position_estimation->local_position);
		}
		waypoint_handler_nav_plan_init(navigation->waypoint_handler);
	}
}

static void navigation_critical_handler(navigation_t* navigation)
{
	float rel_pos[3];
	bool next_state = false;
	
	//Check whether we entered critical mode due to a battery low level
	if ( (navigation->state->battery.is_low)&&(navigation->critical_behavior != CRITICAL_LAND) )
	{
		navigation->critical_behavior = CRITICAL_LAND;
		navigation->critical_next_state = false;
	}
	
	if (!(navigation->critical_next_state))
	{
		navigation->critical_next_state = true;
		
		aero_attitude_t aero_attitude;
		aero_attitude=coord_conventions_quat_to_aero(*navigation->qe);
		navigation->waypoint_handler->waypoint_critical_coordinates.heading = aero_attitude.rpy[2];
		
		switch (navigation->critical_behavior)
		{
			case CLIMB_TO_SAFE_ALT:
				print_util_dbg_print("Climbing to safe alt...\r\n");
				navigation->state->mav_mode_custom = CUST_CRITICAL_CLIMB_TO_SAFE_ALT;
				navigation->waypoint_handler->waypoint_critical_coordinates.pos[X] = navigation->position_estimation->local_position.pos[X];
				navigation->waypoint_handler->waypoint_critical_coordinates.pos[Y] = navigation->position_estimation->local_position.pos[Y];
				navigation->waypoint_handler->waypoint_critical_coordinates.pos[Z] = -30.0f;
				break;
			
			case FLY_TO_HOME_WP:
				navigation->state->mav_mode_custom = CUST_CRITICAL_FLY_TO_HOME_WP;
				navigation->waypoint_handler->waypoint_critical_coordinates.pos[X] = 0.0f;
				navigation->waypoint_handler->waypoint_critical_coordinates.pos[Y] = 0.0f;
				navigation->waypoint_handler->waypoint_critical_coordinates.pos[Z] = -30.0f;
				break;
			
			case HOME_LAND:
				navigation->state->mav_mode_custom = CUST_CRITICAL_LAND;
				navigation->waypoint_handler->waypoint_critical_coordinates.pos[X] = 0.0f;
				navigation->waypoint_handler->waypoint_critical_coordinates.pos[Y] = 0.0f;
				navigation->waypoint_handler->waypoint_critical_coordinates.pos[Z] = 5.0f;
				navigation->alt_lpf = navigation->position_estimation->local_position.pos[2];
				break;
			
			case CRITICAL_LAND:
				print_util_dbg_print("Critical land...\r\n");
				navigation->state->mav_mode_custom = CUST_CRITICAL_LAND;
				navigation->waypoint_handler->waypoint_critical_coordinates.pos[X] = navigation->position_estimation->local_position.pos[X];
				navigation->waypoint_handler->waypoint_critical_coordinates.pos[Y] = navigation->position_estimation->local_position.pos[Y];
				navigation->waypoint_handler->waypoint_critical_coordinates.pos[Z] = 5.0f;
				navigation->alt_lpf = navigation->position_estimation->local_position.pos[2];
				break;
		}
		
		for (uint8_t i = 0; i < 3; i++)
		{
			rel_pos[i] = navigation->waypoint_handler->waypoint_critical_coordinates.pos[i] - navigation->position_estimation->local_position.pos[i];
		}
		navigation->waypoint_handler->dist2wp_sqr = vectors_norm_sqr(rel_pos);
	}
	
	if (navigation->critical_behavior == CRITICAL_LAND || navigation->critical_behavior == HOME_LAND)
	{
		navigation->alt_lpf = navigation->LPF_gain * navigation->alt_lpf + (1.0f - navigation->LPF_gain) * navigation->position_estimation->local_position.pos[2];
		if ( (navigation->position_estimation->local_position.pos[2] > -0.1f)&&(maths_f_abs(navigation->position_estimation->local_position.pos[2] - navigation->alt_lpf) <= 0.2f) )
		{
			// Disarming
			next_state = true;
		}
	}
	
	if ( (navigation->critical_behavior == CLIMB_TO_SAFE_ALT)||(navigation->critical_behavior == FLY_TO_HOME_WP) )
	{
		if (navigation->waypoint_handler->dist2wp_sqr < 3.0f)
		{
			next_state = true;
		}
	}
	
	if (next_state)
	{
		navigation->critical_next_state = false;
		switch (navigation->critical_behavior)
		{
			case CLIMB_TO_SAFE_ALT:
				print_util_dbg_print("Critical State! Flying to home waypoint.\r\n");
				navigation->critical_behavior = FLY_TO_HOME_WP;
				break;
			
			case FLY_TO_HOME_WP:
				print_util_dbg_print("Critical State! Performing critical landing.\r\n");
				navigation->critical_behavior = HOME_LAND;
				break;
			
			case HOME_LAND:
			case CRITICAL_LAND:
				print_util_dbg_print("Critical State! Landed, switching off motors, Emergency mode.\r\n");
				navigation->critical_behavior = CLIMB_TO_SAFE_ALT;
				navigation->state->mav_mode_custom = CUSTOM_BASE_MODE;
				navigation->state->in_the_air = false;
				navigation->state->mav_mode.ARMED = ARMED_OFF;
				navigation->remote->mode.current_desired_mode.ARMED = ARMED_OFF;
				navigation->state->mav_state = MAV_STATE_EMERGENCY;
				break;
		}
	}
}

static void navigation_auto_landing_handler(navigation_t* navigation)
{
	float rel_pos[3];
	
	bool next_state = false;
	
	if (!navigation->auto_landing_next_state)
	{
		navigation->auto_landing_next_state = true;
		
		switch(navigation->auto_landing_behavior)
		{
			case DESCENT_TO_SMALL_ALTITUDE:
				print_util_dbg_print("Cust: descent to small alt");
				navigation->state->mav_mode_custom = CUST_DESCENT_TO_SMALL_ALTITUDE;
				navigation->waypoint_handler->waypoint_hold_coordinates = navigation->position_estimation->local_position;
				navigation->waypoint_handler->waypoint_hold_coordinates.pos[Z] = -5.0f;
				break;
			
			case DESCENT_TO_GND:
				print_util_dbg_print("Cust: descent to gnd");
				navigation->state->mav_mode_custom = CUST_DESCENT_TO_GND;
				navigation->waypoint_handler->waypoint_hold_coordinates = navigation->position_estimation->local_position;
				navigation->waypoint_handler->waypoint_hold_coordinates.pos[Z] = 0.0f;
				navigation->alt_lpf = navigation->position_estimation->local_position.pos[2];
				break;
		}
		
		for (uint8_t i = 0; i < 3; i++)
		{
			rel_pos[i] = navigation->waypoint_handler->waypoint_critical_coordinates.pos[i] - navigation->position_estimation->local_position.pos[i];
		}
		
		navigation->waypoint_handler->dist2wp_sqr = vectors_norm_sqr(rel_pos);
	}
	
	if (navigation->auto_landing_behavior == DESCENT_TO_GND)
	{
		navigation->alt_lpf = navigation->LPF_gain * navigation->alt_lpf + (1.0f - navigation->LPF_gain) * navigation->position_estimation->local_position.pos[2];
		if ( (navigation->position_estimation->local_position.pos[2] > -0.1f)&&(maths_f_abs(navigation->position_estimation->local_position.pos[2] - navigation->alt_lpf) <= 0.2f) )
		{
			// Disarming
			next_state = true;
		}
	}
	
	if (navigation->auto_landing_behavior == DESCENT_TO_SMALL_ALTITUDE)
	{
		if ( (navigation->waypoint_handler->dist2wp_sqr < 3.0f)&&(abs(navigation->position_estimation->local_position.pos[2] - navigation->waypoint_handler->waypoint_hold_coordinates.pos[2]) < 0.5f) )
		{
			next_state = true;
		}
	}
	
	if (next_state)
	{
		navigation->auto_landing_next_state = false;
		
		switch(navigation->auto_landing_behavior)
		{
			case DESCENT_TO_SMALL_ALTITUDE:
				print_util_dbg_print("Automatic-landing: descent_to_GND\r\n");
				navigation->auto_landing_behavior = DESCENT_TO_GND;
				break;
				
			case DESCENT_TO_GND:
				print_util_dbg_print("Auto-landing: disarming motors \r\n");
				navigation->auto_landing_behavior = DESCENT_TO_SMALL_ALTITUDE;
				navigation->state->mav_mode_custom = CUSTOM_BASE_MODE;
				navigation->waypoint_handler->hold_waypoint_set = false;
				navigation->auto_landing = false;
				navigation->state->in_the_air = false;
				navigation->state->mav_mode.ARMED = ARMED_OFF;
				navigation->remote->mode.current_desired_mode.ARMED = ARMED_OFF;
				navigation->state->mav_state = MAV_STATE_STANDBY;
				break;
		}
	}
}

static mav_result_t navigation_start_stop_navigation(navigation_t* navigation, mavlink_command_long_t* packet)
{
	mav_result_t result = MAV_RESULT_UNSUPPORTED;
	
	if (packet->param1 == MAV_GOTO_DO_HOLD)
	{
		if (packet->param2 == MAV_GOTO_HOLD_AT_CURRENT_POSITION)
		{
			navigation->stop_nav = true;
			navigation_waypoint_hold_init(navigation->waypoint_handler, navigation->position_estimation->local_position);
			
			result = MAV_RESULT_ACCEPTED;
		}
		else if (packet->param2 == MAV_GOTO_HOLD_AT_SPECIFIED_POSITION)
		{
			navigation->stop_nav_there = true;
			
			waypoint_struct_t waypoint;
			
			waypoint.frame = packet->param3;
			waypoint.param4 = packet->param4;
			waypoint.x = packet->param5;
			waypoint.y = packet->param6;
			waypoint.z = packet->param7;
			
			local_coordinates_t waypoint_goal = waypoint_handler_set_waypoint_from_frame(&waypoint,navigation->position_estimation->local_position.origin);
			navigation_waypoint_hold_init(navigation->waypoint_handler, waypoint_goal);
			
			result = MAV_RESULT_ACCEPTED;
		}
	}
	else if (packet->param1 == MAV_GOTO_DO_CONTINUE)
	{
		navigation->stop_nav = false;
		navigation->stop_nav_there = false;
		
		result = MAV_RESULT_ACCEPTED;
	}
	
	return result;
}

static void navigation_stopping_handler(navigation_t* navigation)
{
	float dist2wp_sqr;
	float rel_pos[3];
	
	rel_pos[X] = (float)(navigation->waypoint_handler->waypoint_hold_coordinates.pos[X] - navigation->position_estimation->local_position.pos[X]);
	rel_pos[Y] = (float)(navigation->waypoint_handler->waypoint_hold_coordinates.pos[Y] - navigation->position_estimation->local_position.pos[Y]);
	rel_pos[Z] = (float)(navigation->waypoint_handler->waypoint_hold_coordinates.pos[Z] - navigation->position_estimation->local_position.pos[Z]);
	
	dist2wp_sqr = vectors_norm_sqr(rel_pos);
	if (dist2wp_sqr < 25.0f )
	{
		navigation->stop_nav = true;
	}
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool navigation_init(navigation_t* navigation, navigation_config_t* nav_config, control_command_t* controls_nav, const quat_t* qe, mavlink_waypoint_handler_t* waypoint_handler, const position_estimation_t* position_estimation, state_t* state, const joystick_parsing_t* joystick, remote_t* remote, mavlink_communication_t* mavlink_communication)
{
	bool init_success = true;
	
	//navigation pointer init
	navigation->controls_nav = controls_nav;
	navigation->qe = qe;
	navigation->waypoint_handler = waypoint_handler;
	navigation->position_estimation = position_estimation;
	navigation->state = state;
	navigation->mavlink_stream = &mavlink_communication->mavlink_stream;
	navigation->joystick = joystick;
	navigation->remote = remote;
	
	//navigation controller init
	navigation->controls_nav->rpy[ROLL] = 0.0f;
	navigation->controls_nav->rpy[PITCH] = 0.0f;
	navigation->controls_nav->rpy[YAW] = 0.0f;
	navigation->controls_nav->tvel[X] = 0.0f;
	navigation->controls_nav->tvel[Y] = 0.0f;
	navigation->controls_nav->tvel[Z] = 0.0f;
	navigation->controls_nav->theading = 0.0f;
	navigation->controls_nav->thrust = -1.0f;
	navigation->controls_nav->control_mode = VELOCITY_COMMAND_MODE;
	navigation->controls_nav->yaw_mode = YAW_ABSOLUTE;
	
	navigation->wpt_nav_controller = nav_config->wpt_nav_controller;
	navigation->hovering_controller = nav_config->hovering_controller;
	
	navigation->mode.byte = state->mav_mode.byte;
	
	navigation->auto_takeoff = false;
	navigation->auto_landing = false;
	
	navigation->stop_nav = false;
	navigation->stop_nav_there = false;
	
	navigation->critical_behavior = CLIMB_TO_SAFE_ALT;
	navigation->auto_landing_behavior = DESCENT_TO_SMALL_ALTITUDE;
	
	navigation->critical_next_state = false;
	navigation->auto_landing_next_state = false;
	
	navigation->dist2vel_gain = nav_config->dist2vel_gain;
	navigation->cruise_speed = nav_config->cruise_speed;
	navigation->max_climb_rate = nav_config->max_climb_rate;
	
	navigation->soft_zone_size = nav_config->soft_zone_size;
	
	navigation->alt_lpf = nav_config->alt_lpf;
	navigation->LPF_gain = nav_config->LPF_gain;
		
	navigation->loop_count = 0;
	
	navigation->dt = 0.004;
	
	// Add callbacks for waypoint handler commands requests
	mavlink_message_handler_cmd_callback_t callbackcmd;
	
	callbackcmd.command_id = MAV_CMD_NAV_TAKEOFF; // 22
	callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
	callbackcmd.compid_filter = MAV_COMP_ID_ALL;
	callbackcmd.compid_target = MAV_COMP_ID_ALL; // 0
	callbackcmd.function = (mavlink_cmd_callback_function_t)	&navigation_set_auto_takeoff;
	callbackcmd.module_struct =									navigation;
	init_success &= mavlink_message_handler_add_cmd_callback(&mavlink_communication->message_handler, &callbackcmd);
	
	callbackcmd.command_id = MAV_CMD_NAV_LAND; // 21
	callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
	callbackcmd.compid_filter = MAV_COMP_ID_ALL;
	callbackcmd.compid_target = MAV_COMP_ID_ALL; // 0
	callbackcmd.function = (mavlink_cmd_callback_function_t)	&navigation_set_auto_landing;
	callbackcmd.module_struct =									navigation;
	init_success &= mavlink_message_handler_add_cmd_callback(&mavlink_communication->message_handler, &callbackcmd);
	
	callbackcmd.command_id = MAV_CMD_OVERRIDE_GOTO; // 252
	callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
	callbackcmd.compid_filter = MAV_COMP_ID_ALL;
	callbackcmd.compid_target = MAV_COMP_ID_ALL; // 0
	callbackcmd.function = (mavlink_cmd_callback_function_t)	&navigation_start_stop_navigation;
	callbackcmd.module_struct =									navigation;
	init_success &= mavlink_message_handler_add_cmd_callback(&mavlink_communication->message_handler, &callbackcmd);
	
	
	print_util_dbg_print("[NAVIGATION] initialized.\r\n");
	
	return init_success;
}

void navigation_waypoint_hold_init(mavlink_waypoint_handler_t* waypoint_handler, local_coordinates_t local_pos)
{
	waypoint_handler->hold_waypoint_set = true;
	
	waypoint_handler->waypoint_hold_coordinates = local_pos;
	
	//waypoint_handler->waypoint_hold_coordinates.heading = coord_conventions_get_yaw(waypoint_handler->ahrs->qe);
	//waypoint_handler->waypoint_hold_coordinates.heading = local_pos.heading;
	
	print_util_dbg_print("Position hold at: (");
	print_util_dbg_print_num(waypoint_handler->waypoint_hold_coordinates.pos[X],10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(waypoint_handler->waypoint_hold_coordinates.pos[Y],10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(waypoint_handler->waypoint_hold_coordinates.pos[Z],10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num((int32_t)(waypoint_handler->waypoint_hold_coordinates.heading*180.0f/3.14f),10);
	print_util_dbg_print(")\r\n");
	
}

task_return_t navigation_update(navigation_t* navigation)
{
	mav_mode_t mode_local = navigation->state->mav_mode;
	
	uint32_t t = time_keeper_get_time_ticks();
	
	navigation->dt = time_keeper_ticks_to_seconds(t - navigation->last_update);
	navigation->last_update = t;
	
	float thrust;
	
	switch (navigation->state->mav_state)
	{
		case MAV_STATE_STANDBY:
			
			navigation->auto_landing = false;
			navigation->auto_takeoff = false;
			navigation->stop_nav = false;
			navigation->stop_nav_there = false;
			navigation->waypoint_handler->hold_waypoint_set = false;
			navigation->state->mav_mode_custom = CUSTOM_BASE_MODE;
			navigation->critical_behavior = CLIMB_TO_SAFE_ALT;
			navigation->critical_next_state = false;
			navigation->auto_landing_behavior = DESCENT_TO_SMALL_ALTITUDE;
			break;
			
		case MAV_STATE_ACTIVE:
			if((mode_local.byte & 0b11011100) == MAV_MODE_ATTITUDE_CONTROL)
			{
				navigation->auto_landing = false;
				navigation->auto_takeoff = false;
				navigation->stop_nav = false;
				navigation->stop_nav_there = false;
				navigation->state->mav_mode_custom = CUSTOM_BASE_MODE;
				navigation->critical_behavior = CLIMB_TO_SAFE_ALT;
				navigation->critical_next_state = false;
				navigation->auto_landing_behavior = DESCENT_TO_SMALL_ALTITUDE;
			}
			
			if (navigation->state->in_the_air)
			{
				if (navigation->auto_landing)
				{
					if ( (mode_local.AUTO == AUTO_ON) || (mode_local.GUIDED == GUIDED_ON) )
					{
						navigation_auto_landing_handler(navigation);
						navigation_run(navigation->waypoint_handler->waypoint_hold_coordinates,navigation);
						
						if (navigation->auto_landing_behavior == DESCENT_TO_GND)
						{
							// Constant velocity to the ground
							navigation->controls_nav->tvel[Z] = 0.3;
						}
					}
				}
				else
				{
					if(mode_local.AUTO == AUTO_ON)
					{
						navigation_waypoint_navigation_handler(navigation);
						
						if ( navigation->state->nav_plan_active && !navigation->stop_nav)
						{
							if (!navigation->stop_nav_there)
							{
								navigation_run(navigation->waypoint_handler->waypoint_coordinates,navigation);
							}
							else
							{
								navigation_run(navigation->waypoint_handler->waypoint_hold_coordinates,navigation);
								
								navigation_stopping_handler(navigation);
							}
						}
						else
						{
							navigation_run(navigation->waypoint_handler->waypoint_hold_coordinates,navigation);
						}
					}
					else if(mode_local.GUIDED == GUIDED_ON)
					{
						navigation_hold_position_handler(navigation);
						
						navigation_run(navigation->waypoint_handler->waypoint_hold_coordinates,navigation);
						break;
					}
				}
			}
			else
			{
				if (navigation->state->remote_active == 1)
				{
					thrust = remote_get_throttle(navigation->remote);
				}
				else
				{
					thrust = joystick_parsing_get_throttle(navigation->joystick);
				}
				
				if (thrust > -0.7f)
				{
					if ((mode_local.GUIDED == GUIDED_ON)||(mode_local.AUTO == AUTO_ON))
					{
						if (!navigation->auto_takeoff)
						{
							navigation->waypoint_handler->hold_waypoint_set = false;
						}
						navigation->auto_takeoff = true;
					}
					else
					{
						navigation->state->in_the_air = true;
					}
				}
				
				if ((mode_local.GUIDED == GUIDED_ON)||(mode_local.AUTO == AUTO_ON))
				{
					if (navigation->auto_takeoff)
					{
						navigation_waypoint_take_off_handler(navigation);
						
						navigation_run(navigation->waypoint_handler->waypoint_hold_coordinates,navigation);
					}
				}
			}
			break;

		case MAV_STATE_CRITICAL:
			// In MAV_MODE_VELOCITY_CONTROL, MAV_MODE_POSITION_HOLD and MAV_MODE_GPS_NAVIGATION
			if (mode_local.STABILISE == STABILISE_ON)
			{
				if (navigation->state->in_the_air)
				{
					navigation_critical_handler(navigation);
					navigation_run(navigation->waypoint_handler->waypoint_critical_coordinates,navigation);
				}
			}
			break;
			
		default:
			break;
	}
	
	navigation->mode = mode_local;
	
	return TASK_RUN_SUCCESS;
}
