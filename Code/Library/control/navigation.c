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

#define KP_YAW 0.2f

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief					Computes the relative position and distance to the given way point
 *
 * \param	waypointPos		Local coordinates of the waypoint
 * \param	rel_pos			Array to store the relative 3D position of the waypoint
 * \param	localPos		The 3D array of the actual position
 *
 * \return					Distance to waypoint squared
 */
static float navigation_set_rel_pos_n_dist2wp(float waypointPos[], float rel_pos[], float localPos[3]);


/**
 * \brief					Sets the Robot speed to reach waypoint
 *
 * \param	rel_pos			Relative position of the waypoint
 * \param	navigationData	The structure of navigation data
 */
static void navigation_set_speed_command(float rel_pos[], navigation_t* navigationData);

/**
 * \brief					Performs velocity-based collision avoidance strategy
 *
 * \param	navigationData	The structure of navigation data
 */
static void navigation_collision_avoidance(navigation_t* navigationData);

/**
 * \brief						Navigates the robot towards waypoint waypoint_input in 3D velocity command mode
 *
 * \param	waypoint_input		Destination waypoint in local coordinate system
 * \param	navigationData		The navigation structure
 */
void navigation_run(local_coordinates_t waypoint_input, navigation_t* navigationData);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static float navigation_set_rel_pos_n_dist2wp(float waypointPos[], float rel_pos[], float localPos[3])
{
	float dist2wp_sqr;
	
	rel_pos[X] = (float)(waypointPos[X] - localPos[X]);
	rel_pos[Y] = (float)(waypointPos[Y] - localPos[Y]);
	rel_pos[Z] = (float)(waypointPos[Z] - localPos[Z]);
	
	dist2wp_sqr = vectors_norm_sqr(rel_pos);
	
	return dist2wp_sqr;
}


static void navigation_set_speed_command(float rel_pos[], navigation_t* navigationData)
{
	float  norm_rel_dist, v_desired;
	UQuat_t qtmp1, qtmp2;
	
	float dir_desired_bf[3];
	// dir_desired[3],
	
	float rel_heading;
	
	norm_rel_dist = sqrt(navigationData->waypoint_handler->dist2wp_sqr);
	
	if (norm_rel_dist < 0.0005f)
	{
		norm_rel_dist += 0.0005f;
	}
	
	// calculate dir_desired in local frame
	// vel = qe-1 * rel_pos * qe
	qtmp1 = quaternions_create_from_vector(rel_pos);
	qtmp2 = quaternions_global_to_local(*navigationData->qe,qtmp1);
	dir_desired_bf[0] = qtmp2.v[0]; dir_desired_bf[1] = qtmp2.v[1]; dir_desired_bf[2] = qtmp2.v[2];
	
	dir_desired_bf[2] = rel_pos[2];
	
	if (((maths_f_abs(rel_pos[X])<=1.0f)&&(maths_f_abs(rel_pos[Y])<=1.0f))||((maths_f_abs(rel_pos[X])<=5.0f)&&(maths_f_abs(rel_pos[Y])<=5.0f)&&(maths_f_abs(rel_pos[Z])>=3.0f)))
	{
		rel_heading = 0.0f;
	}
	else
	{
		rel_heading = maths_calc_smaller_angle(atan2(rel_pos[Y],rel_pos[X]) - navigationData->position_estimator->localPosition.heading);
	}
	
	v_desired = maths_f_min(navigationData->cruise_speed,(maths_center_window_2(4.0f * rel_heading) * navigationData->dist2vel_gain * maths_soft_zone(norm_rel_dist,navigationData->softZoneSize)));
	
	if (v_desired *  maths_f_abs(dir_desired_bf[Z]) > navigationData->max_climb_rate * norm_rel_dist ) {
		v_desired = navigationData->max_climb_rate * norm_rel_dist /maths_f_abs(dir_desired_bf[Z]);
	}
	
	dir_desired_bf[X] = v_desired * dir_desired_bf[X] / norm_rel_dist;
	dir_desired_bf[Y] = v_desired * dir_desired_bf[Y] / norm_rel_dist;
	dir_desired_bf[Z] = v_desired * dir_desired_bf[Z] / norm_rel_dist;
	
	/*
	loopCount = loopCount++ %50;
	if (loopCount == 0)
	{
		mavlink_msg_named_value_float_send(MAVLINK_COMM_0,time_keeper_get_millis(),"v_desired",v_desired*100);
		mavlink_msg_named_value_float_send(MAVLINK_COMM_0,time_keeper_get_millis(),"act_vel",vector_norm(navigationData->position_estimator->vel_bf)*100);
		print_util_dbg_print("Desired_vel_Bf(x100): (");
		print_util_dbg_print_num(dir_desired_bf[X] * 100,10);
		print_util_dbg_print_num(dir_desired_bf[Y] * 100,10);
		print_util_dbg_print_num(dir_desired_bf[Z] * 100,10);
		print_util_dbg_print("). \n");
		print_util_dbg_print("Actual_vel_bf(x100): (");
		print_util_dbg_print_num(navigationData->position_estimator->vel_bf[X] * 100,10);
		print_util_dbg_print_num(navigationData->position_estimator->vel_bf[Y] * 100,10);
		print_util_dbg_print_num(navigationData->position_estimator->vel_bf[Z] * 100,10);
		print_util_dbg_print("). \n");
		print_util_dbg_print("Actual_pos(x100): (");
		print_util_dbg_print_num(navigationData->position_estimator->localPosition.pos[X] * 100,10);
		print_util_dbg_print_num(navigationData->position_estimator->localPosition.pos[Y] * 100,10);
		print_util_dbg_print_num(navigationData->position_estimator->localPosition.pos[Z] * 100,10);
		print_util_dbg_print("). \n");
	}
	*/

	navigationData->controls_nav->tvel[X] = dir_desired_bf[X];
	navigationData->controls_nav->tvel[Y] = dir_desired_bf[Y];
	navigationData->controls_nav->tvel[Z] = dir_desired_bf[Z];		
	navigationData->controls_nav->rpy[YAW] = KP_YAW * rel_heading;
}

static void navigation_collision_avoidance(navigation_t* navigationData)
{
	float new_velocity[3];
	float rel_heading;
	
	// Implement other velocity-based collision avoidance strategy here
	orca_computeNewVelocity(navigationData->orcaData, navigationData->controls_nav->tvel, new_velocity);
	
	if (((maths_f_abs(new_velocity[X])<=1.0f)&&(maths_f_abs(new_velocity[Y])<=1.0f))||((maths_f_abs(new_velocity[X])<=5.0f)&&(maths_f_abs(new_velocity[Y])<=5.0f)&&(maths_f_abs(new_velocity[Z])>=3.0f)))
	{
		rel_heading = 0.0f;
	}
	if (navigationData->waypoint_handler->collision_avoidance)
	{
		rel_heading = maths_calc_smaller_angle(atan2(new_velocity[Y],new_velocity[X]) - navigationData->position_estimator->localPosition.heading);
	}
	
	navigationData->controls_nav->tvel[X] = new_velocity[X];
	navigationData->controls_nav->tvel[Y] = new_velocity[Y];
	navigationData->controls_nav->tvel[Z] = new_velocity[Z];
	navigationData->controls_nav->rpy[YAW] = KP_YAW * rel_heading;
}

void navigation_run(local_coordinates_t waypoint_input, navigation_t* navigationData)
{
	float rel_pos[3];
	
	// Control in translational speed of the platform
	navigationData->waypoint_handler->dist2wp_sqr = navigation_set_rel_pos_n_dist2wp(waypoint_input.pos,
	rel_pos,
	navigationData->position_estimator->localPosition.pos);
	navigation_set_speed_command(rel_pos, navigationData);
	
	if (navigationData->waypoint_handler->collision_avoidance)
	{
		navigation_collision_avoidance(navigationData);
	}
	
	navigationData->controls_nav->theading=waypoint_input.heading;
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void navigation_init(navigation_t* navigationData, Control_Command_t* controls_nav, UQuat_t* qe, mavlink_waypoint_handler_t* waypoint_handler, position_estimator_t* position_estimator, orca_t* orcaData, state_structure_t* state_structure)
{
	
	navigationData->controls_nav = controls_nav;
	navigationData->qe = qe;
	navigationData->waypoint_handler = waypoint_handler;
	navigationData->position_estimator = position_estimator;
	navigationData->orcaData = orcaData;
	navigationData->state_structure = state_structure;
	
	navigationData->controls_nav->rpy[ROLL] = 0.0f;
	navigationData->controls_nav->rpy[PITCH] = 0.0f;
	navigationData->controls_nav->rpy[YAW] = 0.0f;
	navigationData->controls_nav->tvel[X] = 0.0f;
	navigationData->controls_nav->tvel[Y] = 0.0f;
	navigationData->controls_nav->tvel[Z] = 0.0f;
	navigationData->controls_nav->theading = 0.0f;
	navigationData->controls_nav->thrust = -1.0f;
	
	navigationData->dist2vel_gain = 0.7f;
	navigationData->cruise_speed = 3.0f;
	navigationData->max_climb_rate = 1.0f;
	navigationData->softZoneSize = 0.0f;
	
	navigationData->loopCount = 0;
	
	print_util_dbg_print("Navigation initialized.\n");
}

task_return_t navigation_update(navigation_t* navigationData)
{
	switch (navigationData->state_structure->mav_state)
	{
		case MAV_STATE_STANDBY:
		if (((navigationData->state_structure->mav_mode == MAV_MODE_GUIDED_ARMED)||(navigationData->state_structure->mav_mode == MAV_MODE_AUTO_ARMED)) && !navigationData->waypoint_handler->automatic_take_off)
		{
			navigation_run(navigationData->waypoint_handler->waypoint_hold_coordinates,navigationData);
		}
		break;

		case MAV_STATE_ACTIVE:
			switch (navigationData->state_structure->mav_mode)
			{
				case MAV_MODE_GPS_NAVIGATION:
				case MAV_MODE_COLLISION_AVOIDANCE:
					if (navigationData->waypoint_handler->waypoint_set)
					{
						navigation_run(navigationData->waypoint_handler->waypoint_coordinates,navigationData);
					}
					else
					{
						navigation_run(navigationData->waypoint_handler->waypoint_hold_coordinates,navigationData);
					}
					break;

				case MAV_MODE_POSITION_HOLD:
					navigation_run(navigationData->waypoint_handler->waypoint_hold_coordinates,navigationData);
					break;
			
				default:
					break;
			}
			break;

		case MAV_STATE_CRITICAL:
		if ((navigationData->state_structure->mav_mode == MAV_MODE_POSITION_HOLD)||(navigationData->state_structure->mav_mode == MAV_MODE_GPS_NAVIGATION))
		{
			navigation_run(navigationData->waypoint_handler->waypoint_critical_coordinates,navigationData);
		}
		break;
	}
	
	return TASK_RUN_SUCCESS;
}