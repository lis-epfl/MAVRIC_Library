/*
* navigation.c
*
* Created: 13.08.2013 11:57:38
*  Author: ndousse
*/

#include "navigation.h"
#include "boardsupport.h"

#include <math.h>

#define V_TRANSITION_SQR 1

#define KP_ROLL 0.6
#define KP_PITCH -0.6
#define KP_YAW 0.4

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
waypoint_struct current_waypoint;
bool waypoint_set;
bool waypoint_reached;

board_hardware_t *board;

void init_nav()
{
	int8_t i;
	
	board = get_board_hardware();
	
	waypoint_set = false;
	waypoint_reached = false;
	
	if (board->number_of_waypoints > 0 && board->init_gps_position)
	{
		for (i=0;i<board->number_of_waypoints;i++)
		{
			if (board->waypoint_list[i].current == 1)
			{
				board->current_wp = i;
				current_waypoint = board->waypoint_list[board->current_wp];
				set_waypoint_from_frame(current_waypoint);
				
				waypoint_set = true;
				waypoint_reached = false;
				break;
			}
		}
	}
}

void set_waypoint_from_frame(waypoint_struct current_wp)
{
	switch(current_wp.frame)
	{
		case MAV_FRAME_GLOBAL:
		waypoint_coordinates.origin.latitude =current_wp.x;
		waypoint_coordinates.origin.longitude =current_wp.y;
		waypoint_coordinates.origin.altitude =current_wp.z;
		waypoint_coordinates = global_to_local_position(waypoint_coordinates.origin,board->imu1.attitude.localPosition.origin);
		break;
		case MAV_FRAME_LOCAL_NED:
		waypoint_coordinates.pos[X] = current_wp.x;
		waypoint_coordinates.pos[Y] = current_wp.y;
		waypoint_coordinates.pos[Z] = current_wp.z;
		waypoint_coordinates.origin = local_to_global_position(waypoint_coordinates);
		break;
		case MAV_FRAME_MISSION:
		case MAV_FRAME_GLOBAL_RELATIVE_ALT:
		//mavlink_msg_mission_ack_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,MAV_RESULT_UNSUPPORTED);
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
	if (waypoint_set)
	{
		
		rel_pos[0] = waypoint_coordinates.pos[0] - board->imu1.attitude.localPosition.pos[0];
		rel_pos[1] = waypoint_coordinates.pos[1] - board->imu1.attitude.localPosition.pos[1];
		rel_pos[2] = waypoint_coordinates.pos[2] - board->imu1.attitude.localPosition.pos[2];
		
		dist2wp_sqr = rel_pos[0]*rel_pos[0] + rel_pos[1]*rel_pos[1] + rel_pos[2]*rel_pos[2];
		
		if (dist2wp_sqr >= (current_waypoint.param2*current_waypoint.param2))
		{
			if (current_waypoint.autocontinue == 1)
			{
				if (board->current_wp == board->number_of_waypoints)
				{
					board->current_wp = 0;
				}else{
					board->current_wp++;
				}
				current_waypoint = board->waypoint_list[board->current_wp];
				set_waypoint_from_frame(current_waypoint);
			}else{
				// set speeds to zero
				waypoint_set = false;
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
	float vel_sqr_norm, norm_rel_dist, v_desired;
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
	
	vel_sqr_norm = board->imu1.attitude.vel_bf[0]*board->imu1.attitude.vel_bf[0] + board->imu1.attitude.vel_bf[1]*board->imu1.attitude.vel_bf[1] + board->imu1.attitude.vel_bf[2]*board->imu1.attitude.vel_bf[2];
	
	if (vel_sqr_norm <= V_TRANSITION_SQR)
	{
		low_speed_nav(dir_desired_bf,board->imu1.attitude);
	}else{
		high_speed_nav(dir_desired_bf,board->imu1.attitude);
	}
}

void low_speed_nav(float dir_desired_bf[], Quat_Attitude_t attitude)
{
	if (atan2(dir_desired_bf[Y],dir_desired_bf[X])>(PI/10.0))
	{
		board->controls.rpy[ROLL] = set_roll(0.0, attitude.vel_bf[Y]);
		board->controls.rpy[PITCH] = set_pitch(0.0, attitude.vel_bf[X]);
	}else{
		board->controls.rpy[ROLL] = set_roll(dir_desired_bf[Y], attitude.vel_bf[Y]);
		board->controls.rpy[PITCH] = set_pitch(dir_desired_bf[X], attitude.vel_bf[X]);
	}
	board->controls.rpy[YAW] = set_yaw(dir_desired_bf[Y], dir_desired_bf[X]);
}

void high_speed_nav(float dir_desired_bf[], Quat_Attitude_t attitude)
{
	board->controls.rpy[ROLL] = set_roll(dir_desired_bf[Y], attitude.vel_bf[Y]);
	board->controls.rpy[PITCH] = set_pitch(dir_desired_bf[X], attitude.vel_bf[X]);
	board->controls.rpy[YAW] = set_yaw(attitude.vel_bf[Y], attitude.vel_bf[X]);
	
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
	pitch_set = min_max_bound(pitch_set,MIN_PITCH_RATE,MIN_PITCH_RATE);
	
	return pitch_set;
}

float set_yaw(float value_x, float value_y)
{
	float yaw_set;
	
	yaw_set = KP_YAW * atan2(value_y, value_x);
	
	//yaw_set = turn_force - 0.95 * rate;
	
	yaw_set = min_max_bound(yaw_set,MIN_YAW_RATE,MAX_YAW_RATE);
	
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