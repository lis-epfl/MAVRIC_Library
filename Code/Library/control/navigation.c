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
 * \file navigation.c
 * 
 * Waypoint navigation controller 
 */


#include "navigation.h"
#include "conf_platform.h"
#include "print_util.h"
#include "time_keeper.h"

#define KP_YAW 0.2f

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

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
 * \brief					Performs velocity-based collision avoidance strategy
 *
 * \param	navigation	The structure of navigation data
 */
static void navigation_collision_avoidance(navigation_t* navigation);

/**
 * \brief						Navigates the robot towards waypoint waypoint_input in 3D velocity command mode
 *
 * \param	waypoint_input		Destination waypoint in local coordinate system
 * \param	navigation		The navigation structure
 */
static void navigation_run(local_coordinates_t waypoint_input, navigation_t* navigation);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

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
	float  norm_rel_dist, v_desired;
	UQuat_t qtmp1, qtmp2;
	
	float dir_desired_bf[3];
	// dir_desired[3],
	
	float rel_heading;
	
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
	
	if (((maths_f_abs(rel_pos[X])<=1.0f)&&(maths_f_abs(rel_pos[Y])<=1.0f))||((maths_f_abs(rel_pos[X])<=5.0f)&&(maths_f_abs(rel_pos[Y])<=5.0f)&&(maths_f_abs(rel_pos[Z])>=3.0f)))
	{
		rel_heading = 0.0f;
	}
	else
	{
		rel_heading = maths_calc_smaller_angle(atan2(rel_pos[Y],rel_pos[X]) - navigation->position_estimator->local_position.heading);
	}
	
	v_desired = maths_f_min(navigation->cruise_speed,(maths_center_window_2(4.0f * rel_heading) * navigation->dist2vel_gain * maths_soft_zone(norm_rel_dist,navigation->soft_zone_size)));
	
	if (v_desired *  maths_f_abs(dir_desired_bf[Z]) > navigation->max_climb_rate * norm_rel_dist ) {
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
		mavlink_msg_named_value_float_send(MAVLINK_COMM_0,time_keeper_get_millis(),"act_vel",vector_norm(navigation->position_estimator->vel_bf)*100);
		print_util_dbg_print("Desired_vel_Bf(x100): (");
		print_util_dbg_print_num(dir_desired_bf[X] * 100,10);
		print_util_dbg_print_num(dir_desired_bf[Y] * 100,10);
		print_util_dbg_print_num(dir_desired_bf[Z] * 100,10);
		print_util_dbg_print("). \n");
		print_util_dbg_print("Actual_vel_bf(x100): (");
		print_util_dbg_print_num(navigation->position_estimator->vel_bf[X] * 100,10);
		print_util_dbg_print_num(navigation->position_estimator->vel_bf[Y] * 100,10);
		print_util_dbg_print_num(navigation->position_estimator->vel_bf[Z] * 100,10);
		print_util_dbg_print("). \n");
		print_util_dbg_print("Actual_pos(x100): (");
		print_util_dbg_print_num(navigation->position_estimator->local_position.pos[X] * 100,10);
		print_util_dbg_print_num(navigation->position_estimator->local_position.pos[Y] * 100,10);
		print_util_dbg_print_num(navigation->position_estimator->local_position.pos[Z] * 100,10);
		print_util_dbg_print("). \n");
	}
	*/

	navigation->controls_nav->tvel[X] = dir_desired_bf[X];
	navigation->controls_nav->tvel[Y] = dir_desired_bf[Y];
	navigation->controls_nav->tvel[Z] = dir_desired_bf[Z];		
	navigation->controls_nav->rpy[YAW] = KP_YAW * rel_heading;
}

static void navigation_collision_avoidance(navigation_t* navigation)
{
	float new_velocity[3];
	float rel_heading;
	
	// Implement other velocity-based collision avoidance strategy here
	orca_computeNewVelocity(navigation->orca, navigation->controls_nav->tvel, new_velocity);
	
	if (((maths_f_abs(new_velocity[X])<=1.0f)&&(maths_f_abs(new_velocity[Y])<=1.0f))||((maths_f_abs(new_velocity[X])<=5.0f)&&(maths_f_abs(new_velocity[Y])<=5.0f)&&(maths_f_abs(new_velocity[Z])>=3.0f)))
	{
		rel_heading = 0.0f;
	}
	
	if (navigation->collision_avoidance)
	{		
		rel_heading = maths_calc_smaller_angle(atan2(new_velocity[Y],new_velocity[X]) - navigation->position_estimator->local_position.heading);
	}
	
	navigation->controls_nav->tvel[X] = new_velocity[X];
	navigation->controls_nav->tvel[Y] = new_velocity[Y];
	navigation->controls_nav->tvel[Z] = new_velocity[Z];
	navigation->controls_nav->rpy[YAW] = KP_YAW * rel_heading;
}

static void navigation_run(local_coordinates_t waypoint_input, navigation_t* navigation)
{
	float rel_pos[3];
	
	// Control in translational speed of the platform
	navigation->waypoint_handler->dist2wp_sqr = navigation_set_rel_pos_n_dist2wp(waypoint_input.pos,
																					rel_pos,
																					navigation->position_estimator->local_position.pos);
	navigation_set_speed_command(rel_pos, navigation);
	
	if (navigation->collision_avoidance)	{
		navigation_collision_avoidance(navigation);
	}
	
	navigation->controls_nav->theading=waypoint_input.heading;
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void navigation_init(navigation_t* navigation, control_command_t* controls_nav, const UQuat_t* qe, mavlink_waypoint_handler_t* waypoint_handler, const position_estimator_t* position_estimator, orca_t* orca, const state_t* state, const mavlink_stream_t* mavlink_stream)
{
	
	navigation->controls_nav = controls_nav;
	navigation->qe = qe;
	navigation->waypoint_handler = waypoint_handler;
	navigation->position_estimator = position_estimator;
	navigation->orca = orca;
	navigation->state = state;
	navigation->mavlink_stream = mavlink_stream;
	
	navigation->controls_nav->rpy[ROLL] = 0.0f;
	navigation->controls_nav->rpy[PITCH] = 0.0f;
	navigation->controls_nav->rpy[YAW] = 0.0f;
	navigation->controls_nav->tvel[X] = 0.0f;
	navigation->controls_nav->tvel[Y] = 0.0f;
	navigation->controls_nav->tvel[Z] = 0.0f;
	navigation->controls_nav->theading = 0.0f;
	navigation->controls_nav->thrust = -1.0f;
	
	navigation->dist2vel_gain = 0.7f;
	navigation->cruise_speed = 3.0f;
	navigation->max_climb_rate = 1.0f;
	navigation->soft_zone_size = 0.0f;
	
	navigation->loop_count = 0;
	
	navigation->collision_avoidance = false;
	
	print_util_dbg_print("Navigation initialized.\n");
}

task_return_t navigation_update(navigation_t* navigation)
{
	switch (navigation->state->mav_state)
	{
		case MAV_STATE_STANDBY:
			if (state_test_if_in_flag_mode(navigation->state,MAV_MODE_FLAG_GUIDED_ENABLED)||state_test_if_in_flag_mode(navigation->state,MAV_MODE_FLAG_AUTO_ENABLED))
			{
				waypoint_handler_waypoint_take_off_handler(navigation->waypoint_handler);
				
				navigation_run(navigation->waypoint_handler->waypoint_hold_coordinates,navigation);
			}
			break;

		case MAV_STATE_ACTIVE:
			switch (navigation->state->mav_mode - (navigation->state->mav_mode & MAV_MODE_FLAG_DECODE_POSITION_HIL))
			{
				case MAV_MODE_GPS_NAVIGATION:
					waypoint_handler_waypoint_navigation_handler(navigation->waypoint_handler);
					
					if (navigation->waypoint_handler->waypoint_set)
					{
						navigation_run(navigation->waypoint_handler->waypoint_coordinates,navigation);
					}
					else
					{
						navigation_run(navigation->waypoint_handler->waypoint_hold_coordinates,navigation);
					}
					break;

				case MAV_MODE_POSITION_HOLD:
					waypoint_handler_waypoint_hold_position_handler(navigation->waypoint_handler);
					
					navigation_run(navigation->waypoint_handler->waypoint_hold_coordinates,navigation);
					break;
			
				default:
					break;
			}
			break;

		case MAV_STATE_CRITICAL:
			// In MAV_MODE_VELOCITY_CONTROL, MAV_MODE_POSITION_HOLD and MAV_MODE_GPS_NAVIGATION
			if (state_test_if_in_flag_mode(navigation->state,MAV_MODE_FLAG_STABILIZE_ENABLED))
			{
				waypoint_handler_waypoint_critical_handler(navigation->waypoint_handler);
				navigation_run(navigation->waypoint_handler->waypoint_critical_coordinates,navigation);
			}
			break;
	}
	
	return TASK_RUN_SUCCESS;
}

task_return_t navigation_send_collision_avoidance_status(navigation_t *navigation_data)
{
	mavlink_message_t msg;
	
	mavlink_msg_named_value_int_pack(	navigation_data->mavlink_stream->sysid,
										navigation_data->mavlink_stream->compid,
										&msg,
										time_keeper_get_millis(),
										"Coll_Avoidance",
										navigation_data->collision_avoidance	);
	
	mavlink_stream_send(navigation_data->mavlink_stream,&msg);
	
	return TASK_RUN_SUCCESS;
}