/*
* navigation.c
*
* Created: 13.08.2013 11:57:38
*  Author: ndousse
*/

#include "navigation.h"
#include "boardsupport.h"
#include "print_util.h"

#include <math.h>

#define V_TRANSITION 5
#define V_TRANSITION_SQR V_TRANSITION*V_TRANSITION

#define KP_ROLL 0.6
#define KP_PITCH -0.6
#define KP_YAW 0.2
#define KP_ALT 0.05

#define KD_ALT 0.025
#define KI_ALT 0.0001

#define MIN_ROLL_RATE -2.0
#define MAX_ROLL_RATE 2.0

#define MIN_PITCH_RATE -2.0
#define MAX_PITCH_RATE 2.0

#define MIN_YAW_RATE -2.0
#define MAX_YAW_RATE 2.0

#define V_CRUISE 2.5

#define NAV_HOLD_POS       0
#define NAV_LOW_VELOCITY   1
#define NAV_HIGH_VELOCITY  2

#define SCP(u,v) \
(u[0]*v[0]+u[1]*v[1]+u[2]*v[2]);

#define QI(q, out) \
out.s = q.s;\
out.v[0] = -q.v[0];\
out.v[1] = -q.v[1];\
out.v[2] = -q.v[2];

#define QUAT(q, s, v0, v1, v2) \
q.s=s; q.v[0]=v0; q.v[1]=v1; q.v[2]=v2;

#define QMUL(q1,q2,out) \
tmp[0] = q1.v[1] * q2.v[2] - q1.v[2]*q2.v[1];\
tmp[1] = q1.v[2] * q2.v[0] - q1.v[0]*q2.v[2];\
tmp[2] = q1.v[0] * q2.v[1] - q1.v[1]*q2.v[0];\
out.v[0] = q2.s*q1.v[0] + q1.s *q2.v[0] +tmp[0];\
out.v[1] = q2.s*q1.v[1] + q1.s *q2.v[1] +tmp[1];\
out.v[2] = q2.s*q1.v[2] + q1.s *q2.v[2] +tmp[2];\
out.s= q1.s*q2.s - SCP(q1.v, q2.v);

local_coordinates_t waypoint_coordinates;
global_position_t waypoint_global;
waypoint_struct current_waypoint;
bool waypoint_reached;

board_hardware_t *board;

float alt_integrator;

void init_nav()
{
	int8_t i;
	
	dbg_print("Nav init\n");
	
	board = get_board_hardware();
	
	board->waypoint_set = false;
	waypoint_reached = false;
	
	if (board->number_of_waypoints > 0 && (board->init_gps_position || board->simulation_mode))
	{
		for (i=0;i<board->number_of_waypoints;i++)
		{
			if (board->waypoint_list[i].current == 1)
			{
				board->current_wp = i;
				current_waypoint = board->waypoint_list[board->current_wp];
				set_waypoint_from_frame(current_waypoint);
				
				dbg_print("Waypoint set\n");
				
				board->waypoint_set = true;
				waypoint_reached = false;
				break;
			}
		}
	}
	
	board->controls_nav.rpy[ROLL] = board->controls.rpy[ROLL];
	board->controls_nav.rpy[PITCH] = board->controls.rpy[PITCH];
	board->controls_nav.rpy[YAW] = board->controls.rpy[YAW];
	board->controls_nav.thrust = board->controls.thrust;
	
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
			waypoint_coordinates = global_to_local_position(waypoint_global,board->imu1.attitude.localPosition.origin);
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
			waypoint_global.altitude = -current_wp.z; //Z down
			
			global_position_t origin_relative_alt = board->imu1.attitude.localPosition.origin;
			origin_relative_alt.altitude = 0.0;
			waypoint_coordinates = global_to_local_position(waypoint_global,origin_relative_alt);
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
	// Control in speed of the platform
	if (board->waypoint_set)
	{
		rel_pos[X] = waypoint_coordinates.pos[X] - board->imu1.attitude.localPosition.pos[X];
		rel_pos[Y] = waypoint_coordinates.pos[Y] - board->imu1.attitude.localPosition.pos[Y];
		rel_pos[Z] = waypoint_coordinates.pos[Z] - board->imu1.attitude.localPosition.pos[Z];
		
		dbg_print("rel_pos:(");
		dbg_print_num(rel_pos[X],10);
		dbg_print_num(rel_pos[Y],10);
		dbg_print_num(rel_pos[Z],10);
		dbg_print("\n");
		
		dist2wp_sqr = rel_pos[0]*rel_pos[0] + rel_pos[1]*rel_pos[1] + rel_pos[2]*rel_pos[2];
		
		if (dist2wp_sqr <= (current_waypoint.param2*current_waypoint.param2))
		{
			dbg_print("Waypoint reached, distance:");
			dbg_print_num(sqrt(dist2wp_sqr),10);
			dbg_print(" less than :");
			dbg_print_num(current_waypoint.param2,10);
			dbg_print("\n");
			if (current_waypoint.autocontinue == 1)
			{
				dbg_print("Autocontinue\n");
				if (board->current_wp == board->number_of_waypoints)
				{
					board->current_wp = 0;
				}else{
					board->current_wp++;
				}
				current_waypoint = board->waypoint_list[board->current_wp];
				set_waypoint_from_frame(current_waypoint);
			}else{
				dbg_print("Stop\n");
				// set speeds to zero
				board->waypoint_set = false;
				for (i=0;i<3;i++)
				{
					rel_pos[i] = 0.0;
				}
			}
		}
	}else{
		init_nav();
		for (i=0;i<3;i++)
		{
			rel_pos[i] = 0.0;
		}
	}
	set_speed_command(rel_pos);
}


void set_speed_command(float rel_pos[])
{
	int8_t i;
	float h_vel_sqr_norm, norm_rel_dist, v_desired;
	UQuat_t qtmp1, qtmp2, qtmp3, qtmp4;
	
	float dir_desired_bf[3], dir_desired[3], tmp[3];

	v_desired = min(V_CRUISE,sqrt(rel_pos[0]*rel_pos[0] + rel_pos[1]*rel_pos[1] + rel_pos[2]*rel_pos[2]));
	
	rel_pos[2] = 0;
	norm_rel_dist = sqrt(rel_pos[0]*rel_pos[0] + rel_pos[1]*rel_pos[1] + rel_pos[2]*rel_pos[2])+0.0001;
	
	dir_desired[0] = v_desired * rel_pos[0] / norm_rel_dist;
	dir_desired[1] = v_desired * rel_pos[1] / norm_rel_dist;
	dir_desired[2] = v_desired * rel_pos[2] / norm_rel_dist;

	
	// calculate dir_desired in local frame
	// vel = qe-1 * rel_pos * qe
	qtmp1.s= 0.0; qtmp1.v[0]=dir_desired[0]; qtmp1.v[1]=dir_desired[1]; qtmp1.v[2]=dir_desired[2];
	QI(board->imu1.attitude.qe, qtmp2);
	QMUL(qtmp2, qtmp1, qtmp3);
	QMUL(qtmp3, board->imu1.attitude.qe, qtmp4);
	dir_desired_bf[0] = qtmp4.v[0]; dir_desired_bf[1] = qtmp4.v[1]; dir_desired_bf[2] = qtmp4.v[2];
	
	dir_desired_bf[2] = current_waypoint.z;
	
	h_vel_sqr_norm = board->imu1.attitude.vel_bf[0]*board->imu1.attitude.vel_bf[0] + board->imu1.attitude.vel_bf[1]*board->imu1.attitude.vel_bf[1];
	
	//if (h_vel_sqr_norm <= V_TRANSITION_SQR)
	//{
		//low_speed_nav(dir_desired_bf,board->imu1.attitude);
	//}else{
		//high_speed_nav(dir_desired_bf,board->imu1.attitude);
	//}
	
	
	//low_speed_nav(dir_desired_bf,board->imu1.attitude);
	altitude_nav(dir_desired_bf[Z]);
}

void low_speed_nav(float dir_desired_bf[], Quat_Attitude_t attitude)
{
	// dbg_print("Low speed nav\n");
	//if (atan2(dir_desired_bf[Y],dir_desired_bf[X])>=(PI/10.0))
	//{
		//board->controls_nav.rpy[ROLL] = set_roll(0.0, attitude.vel_bf[Y]);
		//board->controls_nav.rpy[PITCH] = set_pitch(0.0, attitude.vel_bf[X]);
	//}else{
		//board->controls_nav.rpy[ROLL] = set_roll(dir_desired_bf[Y], attitude.vel_bf[Y]);
		//board->controls_nav.rpy[PITCH] = set_pitch(dir_desired_bf[X], attitude.vel_bf[X]);
	//}
	//board->controls_nav.rpy[YAW] = set_yaw(dir_desired_bf[X], dir_desired_bf[Y]);
	
	board->controls_nav.rpy[ROLL] = set_roll(0.0, attitude.vel_bf[Y]);
	board->controls_nav.rpy[PITCH] = set_pitch(0.0, attitude.vel_bf[X]);
	board->controls_nav.rpy[YAW] = set_yaw(0.0, 0.0);
}

void high_speed_nav(float dir_desired_bf[], Quat_Attitude_t attitude)
{
	dbg_print("High speed nav\n");
	board->controls_nav.rpy[ROLL] = set_roll(dir_desired_bf[Y], attitude.vel_bf[Y]);
	board->controls_nav.rpy[PITCH] = set_pitch(dir_desired_bf[X], attitude.vel_bf[X]);
	board->controls_nav.rpy[YAW] = set_yaw(attitude.vel_bf[X], attitude.vel_bf[Y]);
	
}

void altitude_nav(float dir_desired_bf_z)
{
	int i;
	
	float err;
	
	err = dir_desired_bf_z + board->imu1.attitude.localPosition.pos[Z];
	
	board->controls_nav.thrust = KP_ALT * err + KD_ALT * board->imu1.attitude.vel[Z] + KI_ALT * alt_integrator;
	
	alt_integrator += err; 
	
	alt_integrator = min_max_bound(alt_integrator,-10.0,10.0);
	
	//dbg_print("Thrust:");
	//dbg_print_num(board->controls.thrust*100,10);
	//dbg_print(" = ");
	//dbg_print_num(err,10);
	//dbg_print(" = ");
	//dbg_print_num(dir_desired_bf_z*100,10);
	//dbg_print(", ");
	//dbg_print_num(board->imu1.attitude.localPosition.pos[Z]*100,10);
	//dbg_print(", ");
	//dbg_print_num(alt_integrator*100,10);
	//dbg_print("\n");
}

float set_roll(float direction_bf_y, float vel_bf_y)
{
	float roll_set;
	
	roll_set = KP_ROLL * tanh(0.05 * (direction_bf_y - vel_bf_y));
	roll_set = min_max_bound(roll_set,MIN_ROLL_RATE,MAX_ROLL_RATE);
	
	return roll_set;
}

float set_pitch(float direction_bf_x, float vel_bf_x)
{
	float pitch_set;
	
	pitch_set = KP_PITCH * tanh(0.05 * (direction_bf_x - vel_bf_x));
	pitch_set = min_max_bound(pitch_set,MIN_PITCH_RATE,MAX_PITCH_RATE);
	
	return pitch_set;
}

float set_yaw(float value_x, float value_y)
{
	float yaw_set;
	
	yaw_set = KP_YAW * atan2(value_y, value_x);
	
	//yaw_set = turn_force - 0.95 * rate;
	
	yaw_set = min_max_bound(yaw_set,MIN_YAW_RATE,MAX_YAW_RATE);
	
	//dbg_print("Yaw:");
	//dbg_print_num(yaw_set*100000, 10);
	//dbg_print(" = atan(");
	//dbg_print_num(value_y*100000, 10);
	//dbg_print(", ");
	//dbg_print_num(value_x*100000, 10);
	//dbg_print(")\n");
	
	return yaw_set;
}

float min_max_bound(float value, float min, float max)
{
	if (value >= max)
	{
		return max;
	}else if (value <= min)
	{
		return min;
	}
	return value;
}