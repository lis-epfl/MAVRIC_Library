/*
* navigation.c
*
* Created: 13.08.2013 11:57:38
*  Author: ndousse
*/

#include "navigation.h"
#include "central_data.h"
#include "print_util.h"

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
#define MAX_CLIMB_RATE 3.0

#define MIN_ROLL_RATE -0.5
#define MAX_ROLL_RATE 0.5

#define MIN_PITCH_RATE -0.5
#define MAX_PITCH_RATE 0.5

#define MIN_YAW_RATE -4.0
#define MAX_YAW_RATE 4.0

#define V_CRUISE 5

#define NAV_HOLD_POS       0
#define NAV_LOW_VELOCITY   1
#define NAV_HIGH_VELOCITY  2

#define DIST_2_VEL_GAIN 0.4

local_coordinates_t waypoint_coordinates, waypoint_hold_coordinates;
global_position_t waypoint_global;
waypoint_struct current_waypoint;
bool waypoint_reached;

central_data_t *centralData;
float alt_integrator;

int int_loop_count = 0;

void init_nav()
{
	int8_t i;
	
	if (int_loop_count==0)
	{
		dbg_print("Nav init\n");
	}
	int_loop_count=(int_loop_count+1)%1000;
	
	centralData = get_central_data();
	
	//centralData->waypoint_set = false;
	waypoint_reached = false;
	
	
	if ((centralData->number_of_waypoints > 0) && (centralData->init_gps_position || centralData->simulation_mode) && centralData->waypoint_receiving == false)
	{
		for (i=0;i<centralData->number_of_waypoints;i++)
		{
			if (centralData->waypoint_list[i].current == 1)
			{
				centralData->current_wp = i;
				current_waypoint = centralData->waypoint_list[centralData->current_wp];
				set_waypoint_from_frame(current_waypoint);
				
				dbg_print("Waypoint Nr");
				dbg_print_num(i,10);
				dbg_print(" set,\n");
				
				centralData->waypoint_set = true;
				waypoint_reached = false;
				break;
			}
		}
	}
	
	centralData->controls_nav.tvel[X] = 0.0;
	centralData->controls_nav.tvel[Y] = 0.0;
	centralData->controls_nav.rpy[YAW] = 0.0;
	centralData->controls_nav.tvel[Z] = 0.0; //centralData->controls.thrust;
	
	alt_integrator = 0.0;
}

void set_waypoint_from_frame(waypoint_struct current_wp)
{
	switch(current_wp.frame)
	{
		case MAV_FRAME_GLOBAL:
			waypoint_global.latitude = current_wp.x;
			waypoint_global.longitude = current_wp.y;
			waypoint_global.altitude = current_wp.z;
			waypoint_coordinates = global_to_local_position(waypoint_global,centralData->imu1.attitude.localPosition.origin);
			
			dbg_print("wp_global: lat (x1e7):");
			dbg_print_num(waypoint_global.latitude*10000000,10);
			dbg_print(" long (x1e7):");
			dbg_print_num(waypoint_global.longitude*10000000,10);
			dbg_print(" alt (x1000):");
			dbg_print_num(waypoint_global.altitude*1000,10);
			dbg_print(" wp_coor: x (x100):");
			dbg_print_num(waypoint_coordinates.pos[X]*100,10);
			dbg_print(", y (x100):");
			dbg_print_num(waypoint_coordinates.pos[Y]*100,10);
			dbg_print(", z (x100):");
			dbg_print_num(waypoint_coordinates.pos[Z]*100,10);
			dbg_print(" localOrigin lat (x1e7):");
			dbg_print_num(centralData->imu1.attitude.localPosition.origin.latitude*10000000,10);
			dbg_print(" long (x1e7):");
			dbg_print_num(centralData->imu1.attitude.localPosition.origin.longitude*10000000,10);
			dbg_print(" alt (x1000):");
			dbg_print_num(centralData->imu1.attitude.localPosition.origin.altitude*1000,10);
			dbg_print("\n");
			
			break;
		case MAV_FRAME_LOCAL_NED:
			waypoint_coordinates.pos[X] = current_wp.x;
			waypoint_coordinates.pos[Y] = current_wp.y;
			waypoint_coordinates.pos[Z] = current_wp.z;
			waypoint_coordinates.origin = local_to_global_position(waypoint_coordinates);
			break;
		case MAV_FRAME_MISSION:
			//mavlink_msg_mission_ack_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,MAV_RESULT_UNSUPPORTED);
			break;
		case MAV_FRAME_GLOBAL_RELATIVE_ALT:
			waypoint_global.latitude = current_wp.x;
			waypoint_global.longitude = current_wp.y;
			waypoint_global.altitude = current_wp.z;
			
			global_position_t origin_relative_alt = centralData->imu1.attitude.localPosition.origin;
			origin_relative_alt.altitude = 0.0;
			waypoint_coordinates = global_to_local_position(waypoint_global,origin_relative_alt);
			
			dbg_print("LocalOrigin: lat (x1e7):");
			dbg_print_num(origin_relative_alt.latitude * 10000000,10);
			dbg_print(" long (x1e7):");
			dbg_print_num(origin_relative_alt.longitude * 10000000,10);
			dbg_print(" global alt (x1000):");
			dbg_print_num(centralData->imu1.attitude.localPosition.origin.altitude*1000,10);
			dbg_print(" wp_coor: x (x100):");
			dbg_print_num(waypoint_coordinates.pos[X]*100,10);
			dbg_print(", y (x100):");
			dbg_print_num(waypoint_coordinates.pos[Y]*100,10);
			dbg_print(", z (x100):");
			dbg_print_num(waypoint_coordinates.pos[Z]*100,10);
			dbg_print("\n");
			
			break;
		case MAV_FRAME_LOCAL_ENU:
			//mavlink_msg_mission_ack_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,MAV_RESULT_UNSUPPORTED);
			break;
	}
}

void run_navigation()
{
	int8_t i;
	
	float rel_pos[3], command[3], dist2wp_sqr;
	// Control in translational speed of the platform
	if (centralData->mission_started && centralData->waypoint_set)
	{
		rel_pos[X] = (float)(waypoint_coordinates.pos[X] - centralData->imu1.attitude.localPosition.pos[X]);
		rel_pos[Y] = (float)(waypoint_coordinates.pos[Y] - centralData->imu1.attitude.localPosition.pos[Y]);
		rel_pos[Z] = (float)(waypoint_coordinates.pos[Z] - centralData->imu1.attitude.localPosition.pos[Z]);
		
		dist2wp_sqr = rel_pos[0]*rel_pos[0] + rel_pos[1]*rel_pos[1] + rel_pos[2]*rel_pos[2];
		//set_rel_pos_n_dist2wp(rel_pos,&dist2wp_sqr);
		
		//dbg_print("rel_pos:(");
		//dbg_print_num(rel_pos[X],10);
		//dbg_print_num(rel_pos[Y],10);
		//dbg_print_num(rel_pos[Z],10);
		//dbg_print(")");
		//dbg_print(", wp_coor:(");
		//dbg_print_num(waypoint_coordinates.pos[X],10);
		//dbg_print_num(waypoint_coordinates.pos[Y],10);
		//dbg_print_num(waypoint_coordinates.pos[Z],10);
		//dbg_print(")");
		//dbg_print(", localPos:(");
		//dbg_print_num(centralData->imu1.attitude.localPosition.pos[X],10);
		//dbg_print_num(centralData->imu1.attitude.localPosition.pos[Y],10);
		//dbg_print_num(centralData->imu1.attitude.localPosition.pos[Z],10);
		//dbg_print(")\n");
		
		if (dist2wp_sqr <= (current_waypoint.param2*current_waypoint.param2))
		{
			dbg_print("Waypoint Nr");
			dbg_print_num(centralData->current_wp,10);
			dbg_print(" reached, distance:");
			dbg_print_num(sqrt(dist2wp_sqr),10);
			dbg_print(" less than :");
			dbg_print_num(current_waypoint.param2,10);
			//dbg_print(" reached, distance sqr:");
			//dbg_print_num(dist2wp_sqr,10);
			//dbg_print(" less than sqr:");
			//dbg_print_num(current_waypoint.param2*current_waypoint.param2,10);
			dbg_print("\n");
			mavlink_msg_mission_item_reached_send(MAVLINK_COMM_0,centralData->current_wp);
			
			centralData->waypoint_list[centralData->current_wp].current = 0;
			if (current_waypoint.autocontinue == 1)
			{
				dbg_print("Autocontinue towards waypoint Nr");
				
				if (centralData->current_wp == centralData->number_of_waypoints-1)
				{
					centralData->current_wp = 0;
				}else{
					centralData->current_wp++;
				}
				dbg_print_num(centralData->current_wp,10);
				dbg_print("\n");
				centralData->waypoint_list[centralData->current_wp].current = 1;
				current_waypoint = centralData->waypoint_list[centralData->current_wp];
				set_waypoint_from_frame(current_waypoint);
				
				mavlink_msg_mission_current_send(MAVLINK_COMM_0,centralData->current_wp);
				
			}else{
				centralData->waypoint_set = false;
				dbg_print("Stop\n");
				
				//if (centralData->waypoint_hold_init == 0)
				//{
					//centralData->waypoint_hold_init = true;
					//waypoint_hold_coordinates.pos[X] = centralData->imu1.attitude.localPosition.pos[X];
					//waypoint_hold_coordinates.pos[Y] = centralData->imu1.attitude.localPosition.pos[Y];
					//waypoint_hold_coordinates.pos[Z] = centralData->imu1.attitude.localPosition.pos[Z];
				//}
				wp_hold_init();
			}
			rel_pos[X] = (float)(waypoint_coordinates.pos[X] - centralData->imu1.attitude.localPosition.pos[X]);
			rel_pos[Y] = (float)(waypoint_coordinates.pos[Y] - centralData->imu1.attitude.localPosition.pos[Y]);
			rel_pos[Z] = (float)(waypoint_coordinates.pos[Z] - centralData->imu1.attitude.localPosition.pos[Z]);
			
			dist2wp_sqr = rel_pos[0]*rel_pos[0] + rel_pos[1]*rel_pos[1] + rel_pos[2]*rel_pos[2];
			//set_rel_pos_n_dist2wp(rel_pos,&dist2wp_sqr);
		}
	}else{
		if (centralData->waypoint_set == false)
		{
			init_nav();
		}
		
		//if (centralData->waypoint_hold_init == 0)
		//{
			//dbg_print("Position hold at: ");
			//dbg_print_num(centralData->imu1.attitude.localPosition.pos[X],10);
			//dbg_print_num(centralData->imu1.attitude.localPosition.pos[Y],10);
			//dbg_print_num(centralData->imu1.attitude.localPosition.pos[Z],10);
			//dbg_print(")\n");
			//
			//centralData->waypoint_hold_init = 1;
			//waypoint_hold_coordinates.pos[X] = centralData->imu1.attitude.localPosition.pos[X];
			//waypoint_hold_coordinates.pos[Y] = centralData->imu1.attitude.localPosition.pos[Y];
			//waypoint_hold_coordinates.pos[Z] = centralData->imu1.attitude.localPosition.pos[Z];
		//}
		wp_hold_init();
		
		rel_pos[X] = (float)(waypoint_hold_coordinates.pos[X] - centralData->imu1.attitude.localPosition.pos[X]);
		rel_pos[Y] = (float)(waypoint_hold_coordinates.pos[Y] - centralData->imu1.attitude.localPosition.pos[Y]);
		rel_pos[Z] = (float)(waypoint_hold_coordinates.pos[Z] - centralData->imu1.attitude.localPosition.pos[Z]);
		
		dist2wp_sqr = rel_pos[0]*rel_pos[0] + rel_pos[1]*rel_pos[1] + rel_pos[2]*rel_pos[2];
		//set_rel_pos_n_dist2wp(rel_pos,&dist2wp_sqr);
	}
	set_speed_command(rel_pos,dist2wp_sqr);
}

void wp_hold_init()
{
	if (centralData->waypoint_hold_init == 0)
	{
		dbg_print("Position hold at: ");
		dbg_print_num(centralData->imu1.attitude.localPosition.pos[X],10);
		dbg_print_num(centralData->imu1.attitude.localPosition.pos[Y],10);
		dbg_print_num(centralData->imu1.attitude.localPosition.pos[Z],10);
		dbg_print(")\n");
		
		centralData->waypoint_hold_init = 1;
		waypoint_hold_coordinates.pos[X] = centralData->imu1.attitude.localPosition.pos[X];
		waypoint_hold_coordinates.pos[Y] = centralData->imu1.attitude.localPosition.pos[Y];
		waypoint_hold_coordinates.pos[Z] = centralData->imu1.attitude.localPosition.pos[Z];
	}
}

void set_rel_pos_n_dist2wp(float rel_pos[], float *dist2wp_sqr)
{
	rel_pos[X] = (float)(waypoint_hold_coordinates.pos[X] - centralData->imu1.attitude.localPosition.pos[X]);
	rel_pos[Y] = (float)(waypoint_hold_coordinates.pos[Y] - centralData->imu1.attitude.localPosition.pos[Y]);
	rel_pos[Z] = (float)(waypoint_hold_coordinates.pos[Z] - centralData->imu1.attitude.localPosition.pos[Z]);
	
	*dist2wp_sqr = rel_pos[0]*rel_pos[0] + rel_pos[1]*rel_pos[1] + rel_pos[2]*rel_pos[2];
}

void set_speed_command(float rel_pos[], float dist2wpSqr)
{
	int8_t i;
	float h_vel_sqr_norm, norm_rel_dist, v_desired;
	UQuat_t qtmp1, qtmp2, qtmp3, qtmp4;
	
	float dir_desired_bf[3], dir_desired[3], tmp[3];

	norm_rel_dist = sqrt(dist2wpSqr);

	v_desired = f_min(V_CRUISE,(DIST_2_VEL_GAIN * norm_rel_dist));
	
	if (norm_rel_dist < 0.0005)
	{
		norm_rel_dist += 0.0005;
	}
	
	dir_desired[X] = v_desired * rel_pos[X] / norm_rel_dist;
	dir_desired[Y] = v_desired * rel_pos[Y] / norm_rel_dist;
	dir_desired[Z] = v_desired * rel_pos[Z] / norm_rel_dist;
	
	// calculate dir_desired in local frame
	// vel = qe-1 * rel_pos * qe
	qtmp1 = quat_from_vector(dir_desired);
	qtmp1.s= 0.0; qtmp1.v[0]=dir_desired[0]; qtmp1.v[1]=dir_desired[1]; qtmp1.v[2]=dir_desired[2];
	qtmp2 = quat_global_to_local(centralData->imu1.attitude.qe,qtmp1);
	dir_desired_bf[0] = qtmp2.v[0]; dir_desired_bf[1] = qtmp2.v[1]; dir_desired_bf[2] = qtmp2.v[2];
	
	// experimental: Z-axis in velocity mode is in global frame...
	dir_desired_bf[2] = dir_desired[2];
	
	if (f_abs(dir_desired[Z]) > MAX_CLIMB_RATE)
	{
		dir_desired[X] = dir_desired[X] / f_abs(dir_desired[Z]) * MAX_CLIMB_RATE;
		dir_desired[Y] = dir_desired[Y] / f_abs(dir_desired[Z]) * MAX_CLIMB_RATE;
		dir_desired[Z] = dir_desired[Z] / f_abs(dir_desired[Z]) * MAX_CLIMB_RATE;
	}
	
	//h_vel_sqr_norm = centralData->imu1.attitude.vel_bf[0]*centralData->imu1.attitude.vel_bf[0] + centralData->imu1.attitude.vel_bf[1]*centralData->imu1.attitude.vel_bf[1];
	//if (h_vel_sqr_norm <= V_TRANSITION_SQR)
	//{
		//low_speed_nav(dir_desired_bf,centralData->imu1.attitude);
	//}else{
		//high_speed_nav(dir_desired_bf,centralData->imu1.attitude);
	//}
	
	
	low_speed_nav(dir_desired_bf,centralData->imu1.attitude,norm_rel_dist);

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
	centralData->controls_nav.rpy[YAW] = KP_YAW * atan2(attitude.vel_bf[Y], attitude.vel_bf[X]);
}