/*
* navigation.c
*
*  Created: 13.08.2013 11:57:38
*  Author: ndousse
*/

#include "navigation.h"
#include "central_data.h"
#include "print_util.h"
//#include "orca.h"

#include <math.h>
#include "maths.h"

#define V_TRANSITION 5
#define V_TRANSITION_SQR V_TRANSITION*V_TRANSITION

#define KP_ROLL 1.0
#define KP_PITCH -1.0
#define KP_YAW 0.2
#define KP_ALT -0.02

#define KD_ALT 0.025
#define KI_ALT -0.15

#define ALT_INT_SATURATION_MAX 2.0
#define ALT_INT_SATURATION_MIN -2.0


#define MIN_ROLL_RATE -0.5
#define MAX_ROLL_RATE 0.5

#define MIN_PITCH_RATE -0.5
#define MAX_PITCH_RATE 0.5

#define MIN_YAW_RATE -4.0
#define MAX_YAW_RATE 4.0

#define MAX_CLIMB_RATE 1.0
#define V_CRUISE 3.0

#define NAV_HOLD_POS       0
#define NAV_LOW_VELOCITY   1
#define NAV_HIGH_VELOCITY  2

#define DIST_2_VEL_GAIN 0.3

central_data_t *centralData;
float alt_integrator;

void init_nav()
{
	int8_t i;
	
	centralData = get_central_data();
	
	centralData->controls_nav.tvel[X] = 0.0;
	centralData->controls_nav.tvel[Y] = 0.0;
	centralData->controls_nav.rpy[YAW] = 0.0;
	centralData->controls_nav.tvel[Z] = 0.0; //centralData->controls.thrust;
	
	alt_integrator = 0.0;
}

//void run_navigation()
//{
	//int8_t i;
	//float newVelocity[3];
	//
	//float rel_pos[3], command[3], dist2wp_sqr;
	//// Control in translational speed of the platform
	//if (centralData->mission_started && centralData->waypoint_set)
	//{
		//rel_pos[X] = (float)(waypoint_coordinates.pos[X] - centralData->position_estimator.localPosition.pos[X]);
		//rel_pos[Y] = (float)(waypoint_coordinates.pos[Y] - centralData->position_estimator.localPosition.pos[Y]);
		//rel_pos[Z] = (float)(waypoint_coordinates.pos[Z] - centralData->position_estimator.localPosition.pos[Z]);
		//
		//dist2wp_sqr = rel_pos[0]*rel_pos[0] + rel_pos[1]*rel_pos[1] + rel_pos[2]*rel_pos[2];
		////set_rel_pos_n_dist2wp(waypoint_coordinates.pos,rel_pos,&dist2wp_sqr);
		//
		////dbg_print("rel_pos:(");
		////dbg_print_num(rel_pos[X],10);
		////dbg_print_num(rel_pos[Y],10);
		////dbg_print_num(rel_pos[Z],10);
		////dbg_print(")");
		////dbg_print(", wp_coor:(");
		////dbg_print_num(waypoint_coordinates.pos[X],10);
		////dbg_print_num(waypoint_coordinates.pos[Y],10);
		////dbg_print_num(waypoint_coordinates.pos[Z],10);
		////dbg_print(")");
		////dbg_print(", localPos:(");
		////dbg_print_num(centralData->position_estimator.localPosition.pos[X],10);
		////dbg_print_num(centralData->position_estimator.localPosition.pos[Y],10);
		////dbg_print_num(centralData->position_estimator.localPosition.pos[Z],10);
		////dbg_print(")\n");
		//
		//if (dist2wp_sqr <= (current_waypoint.param2*current_waypoint.param2))
		//{
			//dbg_print("Waypoint Nr");
			//dbg_print_num(centralData->current_wp,10);
			//dbg_print(" reached, distance:");
			//dbg_print_num(sqrt(dist2wp_sqr),10);
			//dbg_print(" less than :");
			//dbg_print_num(current_waypoint.param2,10);
			////dbg_print(" reached, distance sqr:");
			////dbg_print_num(dist2wp_sqr,10);
			////dbg_print(" less than sqr:");
			////dbg_print_num(current_waypoint.param2*current_waypoint.param2,10);
			//dbg_print("\n");
			//mavlink_msg_mission_item_reached_send(MAVLINK_COMM_0,centralData->current_wp);
			//
			//centralData->waypoint_list[centralData->current_wp].current = 0;
			//if (current_waypoint.autocontinue == 1)
			//{
				//dbg_print("Autocontinue towards waypoint Nr");
				//
				//if (centralData->current_wp == centralData->number_of_waypoints-1)
				//{
					//centralData->current_wp = 0;
				//}else{
					//centralData->current_wp++;
				//}
				//dbg_print_num(centralData->current_wp,10);
				//dbg_print("\n");
				//centralData->waypoint_list[centralData->current_wp].current = 1;
				//current_waypoint = centralData->waypoint_list[centralData->current_wp];
				//waypoint_coordinates = set_waypoint_from_frame(current_waypoint,centralData->position_estimator.localPosition.origin);
				//
				//mavlink_msg_mission_current_send(MAVLINK_COMM_0,centralData->current_wp);
				//
			//}else{
				//centralData->waypoint_set = false;
				//dbg_print("Stop\n");
				//
				//wp_hold_init(&centralData->waypoint_hold_init,&waypoint_hold_coordinates, centralData->position_estimator.localPosition);
			//}
			//rel_pos[X] = (float)(waypoint_coordinates.pos[X] - centralData->position_estimator.localPosition.pos[X]);
			//rel_pos[Y] = (float)(waypoint_coordinates.pos[Y] - centralData->position_estimator.localPosition.pos[Y]);
			//rel_pos[Z] = (float)(waypoint_coordinates.pos[Z] - centralData->position_estimator.localPosition.pos[Z]);
			//
			//dist2wp_sqr = rel_pos[0]*rel_pos[0] + rel_pos[1]*rel_pos[1] + rel_pos[2]*rel_pos[2];
			////set_rel_pos_n_dist2wp(waypoint_coordinates.pos,rel_pos,&dist2wp_sqr);
		//}
	//}else{
		//if (centralData->waypoint_set == false)
		//{
			//init_nav();
		//}
		//
		//wp_hold_init(&centralData->waypoint_hold_init,&waypoint_hold_coordinates, centralData->position_estimator.localPosition);
		//
		//rel_pos[X] = (float)(waypoint_hold_coordinates.pos[X] - centralData->position_estimator.localPosition.pos[X]);
		//rel_pos[Y] = (float)(waypoint_hold_coordinates.pos[Y] - centralData->position_estimator.localPosition.pos[Y]);
		//rel_pos[Z] = (float)(waypoint_hold_coordinates.pos[Z] - centralData->position_estimator.localPosition.pos[Z]);
		//
		//dist2wp_sqr = rel_pos[0]*rel_pos[0] + rel_pos[1]*rel_pos[1] + rel_pos[2]*rel_pos[2];
		////set_rel_pos_n_dist2wp(waypoint_hold_coordinates.pos,rel_pos,&dist2wp_sqr);
	//}
	//
	////if (centralData->mav_state == MAV_STATE_CRITICAL)
	////{
		////switch (critical_behavior)
		////{
			////case CLIMB_TO_SAVE_ALT:
				////waypoint_critical_coordinates.pos[X] = centralData->position_estimator.localPosition.pos[X];
				////waypoint_critical_coordinates.pos[Y] = centralData->position_estimator.localPosition.pos[Y];
				////waypoint_critical_coordinates.pos[Z] = -30.0;
				////break;
			////case FLY_TO_HOME_WP:
				////waypoint_critical_coordinates.pos[X] = 0.0;
				////waypoint_critical_coordinates.pos[Y] = 0.0;
				////waypoint_critical_coordinates.pos[Z] = -30.0;
				////break;
			////case CRITICAL_LAND:
				////waypoint_critical_coordinates.pos[X] = 0.0;
				////waypoint_critical_coordinates.pos[Y] = 0.0;
				////waypoint_critical_coordinates.pos[Z] = 0.0;
				////break;
		////}
		////
		////rel_pos[X] = (float)(waypoint_critical_coordinates.pos[X] - centralData->position_estimator.localPosition.pos[X]);
		////rel_pos[Y] = (float)(waypoint_critical_coordinates.pos[Y] - centralData->position_estimator.localPosition.pos[Y]);
		////rel_pos[Z] = (float)(waypoint_critical_coordinates.pos[Z] - centralData->position_estimator.localPosition.pos[Z]);
		////
		////dist2wp_sqr = rel_pos[0]*rel_pos[0] + rel_pos[1]*rel_pos[1] + rel_pos[2]*rel_pos[2];
		////
		//// //set_rel_pos_n_dist2wp(waypoint_critical_coordinates.pos,rel_pos,&dist2wp_sqr);
		////
		////if (dist2wp_sqr < 3.0)
		////{
			////switch (critical_behavior)
			////{
				////case CLIMB_TO_SAVE_ALT:
					////critical_behavior = FLY_TO_HOME_WP;
					////break;
				////case FLY_TO_HOME_WP:
					////critical_behavior = CRITICAL_LAND;
					////break;
				////case CRITICAL_LAND:
					////centralData->critical_landing = true;
					////break;
			////}
		////}
	////}
	//
	//set_speed_command(rel_pos,dist2wp_sqr);
	//centralData->controls_nav.theading=waypoint_hold_coordinates.heading;
	////computeNewVelocity(centralData->controls_nav.tvel,newVelocity);
	//
//}



void run_navigation()
{
	int8_t i;
	float newVelocity[3];
	
	float rel_pos[3], dist2wp_sqr;
	// Control in translational speed of the platform
	if (((centralData->mav_state == MAV_STATE_ACTIVE)||(centralData->mav_state == MAV_STATE_CRITICAL))&&((centralData->mav_mode == MAV_MODE_AUTO_ARMED)||(centralData->mav_mode == MAV_MODE_GUIDED_ARMED)))
	{
		if (centralData->waypoint_set)
		{
			centralData->dist2wp_sqr = set_rel_pos_n_dist2wp(centralData->waypoint_coordinates.pos, rel_pos);
			set_speed_command(rel_pos,centralData->dist2wp_sqr);
			centralData->controls_nav.theading=centralData->waypoint_coordinates.heading;
			//computeNewVelocity(centralData->controls_nav.tvel,newVelocity);
		}
	}
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
	int8_t i;
	float  norm_rel_dist, v_desired;
	UQuat_t qtmp1, qtmp2;
	
	float dir_desired_bf[3], dir_desired[3];
	
	float rel_heading;
	
	norm_rel_dist = sqrt(dist2wpSqr);
	
	if (norm_rel_dist < 0.0005)
	{
		norm_rel_dist += 0.0005;
	}
	

	
	// calculate dir_desired in local frame
	// vel = qe-1 * rel_pos * qe
	qtmp1 = quat_from_vector(rel_pos);
	//qtmp1.s= 0.0; qtmp1.v[0]=dir_desired[0]; qtmp1.v[1]=dir_desired[1]; qtmp1.v[2]=dir_desired[2];
	qtmp2 = quat_global_to_local(centralData->imu1.attitude.qe,qtmp1);
	dir_desired_bf[0] = qtmp2.v[0]; dir_desired_bf[1] = qtmp2.v[1]; dir_desired_bf[2] = qtmp2.v[2];
	
	// experimental: Z-axis in velocity mode is in global frame...
	dir_desired_bf[2] = rel_pos[2];
	
	rel_heading= atan2(dir_desired_bf[Y],dir_desired_bf[X]);
	v_desired = f_min(V_CRUISE,(center_window_2(rel_heading) * DIST_2_VEL_GAIN * norm_rel_dist));
	
	if (v_desired *  f_abs(dir_desired_bf[Z]) > MAX_CLIMB_RATE * norm_rel_dist ) {
		v_desired = MAX_CLIMB_RATE * norm_rel_dist /f_abs(dir_desired_bf[Z]);
	}
	
	dir_desired_bf[X] = v_desired * dir_desired_bf[X] / norm_rel_dist;
	dir_desired_bf[Y] = v_desired * dir_desired_bf[Y] / norm_rel_dist;
	dir_desired_bf[Z] = v_desired * dir_desired_bf[Z] / norm_rel_dist;
	
	
	//h_vel_sqr_norm = centralData->imu1.attitude.vel_bf[0]*centralData->imu1.attitude.vel_bf[0] + centralData->imu1.attitude.vel_bf[1]*centralData->imu1.attitude.vel_bf[1];
	//if (h_vel_sqr_norm <= V_TRANSITION_SQR)
	//{
		//low_speed_nav(dir_desired_bf,centralData->imu1.attitude);
	//}else{
		//high_speed_nav(dir_desired_bf,centralData->imu1.attitude);
	//}
	
		

	centralData->controls_nav.tvel[X] = dir_desired_bf[X];
	centralData->controls_nav.tvel[Y] = dir_desired_bf[Y];
	centralData->controls_nav.tvel[Z] = dir_desired_bf[Z];		
	centralData->controls_nav.rpy[YAW] = KP_YAW * rel_heading;

	//low_speed_nav(dir_desired_bf,centralData->imu1.attitude,norm_rel_dist);

}

void low_speed_nav(float dir_desired_bf[], Quat_Attitude_t attitude, float rel_distance)
{
	// dbg_print("Low speed nav\n");
	
	float yaw_angle_tolerance = PI/10.0;
	
	if ((f_abs(dir_desired_bf[X]) < 0.001 && f_abs(dir_desired_bf[Y]) < 0.001) || centralData->waypoint_hold_init || (rel_distance<=5.0))
	{
		centralData->controls_nav.rpy[YAW] = 0.0;
		centralData->controls_nav.tvel[X] = dir_desired_bf[X];
		centralData->controls_nav.tvel[Y] = dir_desired_bf[Y];
	}else{
		float rel_heading = atan2(dir_desired_bf[Y],dir_desired_bf[X]);
		
		if (rel_heading >= yaw_angle_tolerance)
		{
			centralData->controls_nav.tvel[X] = 0.0;
			centralData->controls_nav.tvel[Y] = 0.0;
		}else{
			centralData->controls_nav.tvel[X] = dir_desired_bf[X];
			centralData->controls_nav.tvel[Y] = dir_desired_bf[Y];
		}
		
		centralData->controls_nav.rpy[YAW] = KP_YAW * rel_heading;
	}

	centralData->controls_nav.tvel[Z] = dir_desired_bf[Z];
}

void high_speed_nav(float dir_desired_bf[], Quat_Attitude_t attitude)
{
	//dbg_print("High speed nav\n");
	
	centralData->controls_nav.tvel[X] = dir_desired_bf[X];
	centralData->controls_nav.tvel[Y] = dir_desired_bf[Y];
	centralData->controls_nav.tvel[Z] = dir_desired_bf[Z];
	centralData->controls_nav.rpy[YAW] = KP_YAW * atan2(centralData->position_estimator.vel_bf[Y], centralData->position_estimator.vel_bf[X]);
}