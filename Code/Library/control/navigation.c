/**
 *  Waypoint navigation controller
 *
 * The MAV'RIC Framework
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 *
 * This file is part of the MAV'RIC Framework.
 */

#include "navigation.h"
#include "central_data.h"
#include "print_util.h"
#include "orca.h"

#include <math.h>
#include "maths.h"

#define KP_YAW 0.2f

central_data_t *centralData;
float alt_integrator;

uint8_t loopCount = 0;

void init_nav(void)
{
//	int8_t i;
	
	centralData = get_central_data();
	
	centralData->controls_nav.tvel[X] = 0.0f;
	centralData->controls_nav.tvel[Y] = 0.0f;
	centralData->controls_nav.rpy[YAW] = 0.0f;
	centralData->controls_nav.tvel[Z] = 0.0f;
	
	alt_integrator = 0.0f;
}


void run_navigation(local_coordinates_t waypoint_input)
{
	// int8_t i;
	// float newVelocity[3];
	
	float rel_pos[3]; 
	// dist2wp_sqr;
	
	// Control in translational speed of the platform
	centralData->dist2wp_sqr = set_rel_pos_n_dist2wp(waypoint_input.pos, rel_pos);
	set_speed_command(rel_pos,centralData->dist2wp_sqr);
	
	centralData->controls_nav.theading=waypoint_input.heading;
}

float set_rel_pos_n_dist2wp(float waypointPos[], float rel_pos[])
{
	float dist2wp_sqr;
	
	rel_pos[X] = (float)(waypointPos[X] - centralData->position_estimator.localPosition.pos[X]);
	rel_pos[Y] = (float)(waypointPos[Y] - centralData->position_estimator.localPosition.pos[Y]);
	rel_pos[Z] = (float)(waypointPos[Z] - centralData->position_estimator.localPosition.pos[Z]);
	
	dist2wp_sqr = vector_norm_sqr(rel_pos);
	
	return dist2wp_sqr;
}

void set_speed_command(float rel_pos[], float dist2wpSqr)
{
	uint8_t i;
	float  norm_rel_dist, v_desired;
	UQuat_t qtmp1, qtmp2;
	
	float dir_desired_bf[3], new_velocity[3];
	// dir_desired[3],
	
	float rel_heading;
	
	norm_rel_dist = sqrt(dist2wpSqr);
	
	if (norm_rel_dist < 0.0005f)
	{
		norm_rel_dist += 0.0005f;
	}
	
	// calculate dir_desired in local frame
	// vel = qe-1 * rel_pos * qe
	qtmp1 = quat_from_vector(rel_pos);
	qtmp2 = quat_global_to_local(centralData->imu1.attitude.qe,qtmp1);
	dir_desired_bf[0] = qtmp2.v[0]; dir_desired_bf[1] = qtmp2.v[1]; dir_desired_bf[2] = qtmp2.v[2];
	
	dir_desired_bf[2] = rel_pos[2];
	
	if (((f_abs(rel_pos[X])<=1.0f)&&(f_abs(rel_pos[Y])<=1.0f))||((f_abs(rel_pos[X])<=5.0f)&&(f_abs(rel_pos[Y])<=5.0f)&&(f_abs(rel_pos[Z])>=3.0f)))
	{
		rel_heading = 0.0f;
	}else{
		rel_heading = calc_smaller_angle(atan2(rel_pos[Y],rel_pos[X]) - centralData->position_estimator.localPosition.heading);
	}
	
	v_desired = f_min(centralData->cruise_speed,(center_window_2(4.0f*rel_heading) * centralData->dist2vel_gain * soft_zone(norm_rel_dist,centralData->softZoneSize)));
	
	if (v_desired *  f_abs(dir_desired_bf[Z]) > centralData->max_climb_rate * norm_rel_dist ) {
		v_desired = centralData->max_climb_rate * norm_rel_dist /f_abs(dir_desired_bf[Z]);
	}
	
	dir_desired_bf[X] = v_desired * dir_desired_bf[X] / norm_rel_dist;
	dir_desired_bf[Y] = v_desired * dir_desired_bf[Y] / norm_rel_dist;
	dir_desired_bf[Z] = v_desired * dir_desired_bf[Z] / norm_rel_dist;
	
	/*
	loopCount = loopCount++ %50;
	if (loopCount == 0)
	{
		mavlink_msg_named_value_float_send(MAVLINK_COMM_0,get_millis(),"v_desired",v_desired*100);
		mavlink_msg_named_value_float_send(MAVLINK_COMM_0,get_millis(),"act_vel",vector_norm(centralData->position_estimator.vel_bf)*100);
		dbg_print("Desired_vel_Bf(x100): (");
		dbg_print_num(dir_desired_bf[X]*100,10);
		dbg_print_num(dir_desired_bf[Y]*100,10);
		dbg_print_num(dir_desired_bf[Z]*100,10);
		dbg_print("). \n");
		dbg_print("Actual_vel_bf(x100): (");
		dbg_print_num(centralData->position_estimator.vel_bf[X]*100,10);
		dbg_print_num(centralData->position_estimator.vel_bf[Y]*100,10);
		dbg_print_num(centralData->position_estimator.vel_bf[Z]*100,10);
		dbg_print("). \n");
		dbg_print("Actual_pos(x100): (");
		dbg_print_num(centralData->position_estimator.localPosition.pos[X]*100,10);
		dbg_print_num(centralData->position_estimator.localPosition.pos[Y]*100,10);
		dbg_print_num(centralData->position_estimator.localPosition.pos[Z]*100,10);
		dbg_print("). \n");
	}
	*/
	
	for (i=0;i<3;i++)
	{
		new_velocity[i] = dir_desired_bf[i];
	}
	if (centralData->collision_avoidance)
	{
		computeNewVelocity(dir_desired_bf,new_velocity);
	}

	//rel_heading= atan2(new_velocity[Y],new_velocity[X]);
	//if (f_abs(new_velocity[X])<0.001f && f_abs(new_velocity[Y])<0.001f || centralData->controls.yaw_mode == YAW_ABSOLUTE || !centralData->waypoint_set)
	//{
		//rel_heading = 0.0f;
	//}else{
		//rel_heading = atan2(new_velocity[Y],new_velocity[X]);
	//}

	centralData->controls_nav.tvel[X] = new_velocity[X];
	centralData->controls_nav.tvel[Y] = new_velocity[Y];
	centralData->controls_nav.tvel[Z] = new_velocity[Z];		
	centralData->controls_nav.rpy[YAW] = KP_YAW * rel_heading;
}