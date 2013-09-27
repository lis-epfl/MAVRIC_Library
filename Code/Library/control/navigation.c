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
#include "maths.h"

#define V_TRANSITION 5
#define V_TRANSITION_SQR V_TRANSITION*V_TRANSITION

#define KP_ROLL 0.6
#define KP_PITCH -0.6
#define KP_YAW 0.05
#define KP_ALT -0.02

#define KD_ALT 0.025
#define KI_ALT -0.15

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


local_coordinates_t waypoint_coordinates, waypoint_hold_coordinates;
global_position_t waypoint_global;
waypoint_struct current_waypoint;
bool waypoint_reached;

bool waypoint_hold_init = false;

board_hardware_t *board;

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
	
	board = get_board_hardware();
	
	//board->waypoint_set = false;
	waypoint_reached = false;
	
	
	if ((board->number_of_waypoints > 0) && (board->init_gps_position || board->simulation_mode) && board->waypoint_receiving == false)
	{
		for (i=0;i<board->number_of_waypoints;i++)
		{
			if (board->waypoint_list[i].current == 1)
			{
				board->current_wp = i;
				current_waypoint = board->waypoint_list[board->current_wp];
				set_waypoint_from_frame(current_waypoint);
				
				dbg_print("Waypoint Nr");
				dbg_print_num(i,10);
				dbg_print(" set,\n");
				
				board->waypoint_set = true;
				waypoint_reached = false;
				break;
			}
		}
	}
	
	board->controls_nav.rpy[ROLL] = 0.0;
	board->controls_nav.rpy[PITCH] = 0.0;
	board->controls_nav.rpy[YAW] = 0.0;
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
			dbg_print_num(board->imu1.attitude.localPosition.origin.latitude*10000000,10);
			dbg_print(" long (x1e7):");
			dbg_print_num(board->imu1.attitude.localPosition.origin.longitude*10000000,10);
			dbg_print(" alt (x1000):");
			dbg_print_num(board->imu1.attitude.localPosition.origin.altitude*1000,10);
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
			
			global_position_t origin_relative_alt = board->imu1.attitude.localPosition.origin;
			origin_relative_alt.altitude = 0.0;
			waypoint_coordinates = global_to_local_position(waypoint_global,origin_relative_alt);
			
			dbg_print("LocalOrigin: lat (x1e7):");
			dbg_print_num(origin_relative_alt.latitude * 10000000,10);
			dbg_print(" long (x1e7):");
			dbg_print_num(origin_relative_alt.longitude * 10000000,10);
			dbg_print(" global alt (x1000):");
			dbg_print_num(board->imu1.attitude.localPosition.origin.altitude*1000,10);
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
	
	double rel_pos[3], command[3], dist2wp_sqr;
	// Control in translational speed of the platform
	if (board->mission_started && board->waypoint_set)
	{
		waypoint_hold_init = false;
		
		rel_pos[X] = waypoint_coordinates.pos[X] - board->imu1.attitude.localPosition.pos[X];
		rel_pos[Y] = waypoint_coordinates.pos[Y] - board->imu1.attitude.localPosition.pos[Y];
		rel_pos[Z] = waypoint_coordinates.pos[Z] - board->imu1.attitude.localPosition.pos[Z];
		
		dbg_print("rel_pos:(");
		dbg_print_num(rel_pos[X],10);
		dbg_print_num(rel_pos[Y],10);
		dbg_print_num(rel_pos[Z],10);
		dbg_print(")");
		dbg_print(", wp_coor:(");
		dbg_print_num(waypoint_coordinates.pos[X],10);
		dbg_print_num(waypoint_coordinates.pos[Y],10);
		dbg_print_num(waypoint_coordinates.pos[Z],10);
		dbg_print(")");
		dbg_print(", localPos:(");
		dbg_print_num(board->imu1.attitude.localPosition.pos[X],10);
		dbg_print_num(board->imu1.attitude.localPosition.pos[Y],10);
		dbg_print_num(board->imu1.attitude.localPosition.pos[Z],10);
		dbg_print(")\n");
		
		dist2wp_sqr = rel_pos[0]*rel_pos[0] + rel_pos[1]*rel_pos[1] + rel_pos[2]*rel_pos[2];
		
		if (dist2wp_sqr <= (current_waypoint.param2*current_waypoint.param2))
		{
			dbg_print("Waypoint Nr");
			dbg_print_num(board->current_wp,10);
			dbg_print(" reached, distance:");
			dbg_print_num(sqrt(dist2wp_sqr),10);
			dbg_print(" less than :");
			dbg_print_num(current_waypoint.param2,10);
			//dbg_print(" reached, distance sqr:");
			//dbg_print_num(dist2wp_sqr,10);
			//dbg_print(" less than sqr:");
			//dbg_print_num(current_waypoint.param2*current_waypoint.param2,10);
			dbg_print("\n");
			mavlink_msg_mission_item_reached_send(MAVLINK_COMM_0,board->current_wp);
			
			board->waypoint_list[board->current_wp].current = 0;
			if (current_waypoint.autocontinue == 1)
			{
				dbg_print("Autocontinue towards waypoint Nr");
				
				if (board->current_wp == board->number_of_waypoints-1)
				{
					board->current_wp = 0;
				}else{
					board->current_wp++;
				}
				dbg_print_num(board->current_wp,10);
				dbg_print("\n");
				board->waypoint_list[board->current_wp].current = 1;
				current_waypoint = board->waypoint_list[board->current_wp];
				set_waypoint_from_frame(current_waypoint);
				
				mavlink_msg_mission_current_send(MAVLINK_COMM_0,board->current_wp);
			}else{
				board->waypoint_set = false;
				dbg_print("Stop\n");
				
				if (waypoint_hold_init == 0)
				{
					waypoint_hold_init = true;
					waypoint_hold_coordinates.pos[X] = board->imu1.attitude.localPosition.pos[X];
					waypoint_hold_coordinates.pos[Y] = board->imu1.attitude.localPosition.pos[Y];
					waypoint_hold_coordinates.pos[Z] = board->imu1.attitude.localPosition.pos[Z];
				}
				
				rel_pos[X] = waypoint_hold_coordinates.pos[X] - board->imu1.attitude.localPosition.pos[X];
				rel_pos[Y] = waypoint_hold_coordinates.pos[Y] - board->imu1.attitude.localPosition.pos[Y];
				rel_pos[Z] = waypoint_hold_coordinates.pos[Z] - board->imu1.attitude.localPosition.pos[Z];
				
				// set speeds to zero
				//for (i=0;i<3;i++)
				//{
					//rel_pos[i] = 0.0;
				//}
			}
		}
	}else{
		if (board->waypoint_set == false)
		{
			init_nav();
		}
		
		if (waypoint_hold_init == 0)
		{
			waypoint_hold_init = 1;
			waypoint_hold_coordinates.pos[X] = board->imu1.attitude.localPosition.pos[X];
			waypoint_hold_coordinates.pos[Y] = board->imu1.attitude.localPosition.pos[Y];
			waypoint_hold_coordinates.pos[Z] = board->imu1.attitude.localPosition.pos[Z];
		}
		
		rel_pos[X] = waypoint_hold_coordinates.pos[X] - board->imu1.attitude.localPosition.pos[X];
		rel_pos[Y] = waypoint_hold_coordinates.pos[Y] - board->imu1.attitude.localPosition.pos[Y];
		rel_pos[Z] = waypoint_hold_coordinates.pos[Z] - board->imu1.attitude.localPosition.pos[Z];
		
	}
	set_speed_command(rel_pos);
}


void set_speed_command(double rel_pos[])
{
	int8_t i;
	double h_vel_sqr_norm, norm_rel_dist, v_desired, z_pos;
	UQuat_t qtmp1, qtmp2, qtmp3, qtmp4;
	
	double dir_desired_bf[3], dir_desired[3], tmp[3];

	v_desired = min(V_CRUISE,sqrt(rel_pos[0]*rel_pos[0] + rel_pos[1]*rel_pos[1] + rel_pos[2]*rel_pos[2]));
	
	z_pos = rel_pos[Z];
	rel_pos[Z] = 0;
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
	
	dir_desired_bf[Z] = z_pos;
	
	h_vel_sqr_norm = board->imu1.attitude.vel_bf[0]*board->imu1.attitude.vel_bf[0] + board->imu1.attitude.vel_bf[1]*board->imu1.attitude.vel_bf[1];
	
	//if (h_vel_sqr_norm <= V_TRANSITION_SQR)
	//{
		//low_speed_nav(dir_desired_bf,board->imu1.attitude);
	//}else{
		//high_speed_nav(dir_desired_bf,board->imu1.attitude);
	//}
	
	low_speed_nav(dir_desired_bf,board->imu1.attitude);
	altitude_nav(dir_desired_bf[Z]);
	//altitude_nav(waypoint_coordinates.pos[Z]);
}

void low_speed_nav(double dir_desired_bf[], Quat_Attitude_t attitude)
{
	// dbg_print("Low speed nav\n");
	if (atan2(dir_desired_bf[Y],dir_desired_bf[X])>=(PI/10.0))
	{
		board->controls_nav.rpy[ROLL] = set_roll(0.0, attitude.vel_bf[Y]);
		board->controls_nav.rpy[PITCH] = set_pitch(0.0, attitude.vel_bf[X]);
	}else{
		board->controls_nav.rpy[ROLL] = set_roll(dir_desired_bf[Y], attitude.vel_bf[Y]);
		board->controls_nav.rpy[PITCH] = set_pitch(dir_desired_bf[X], attitude.vel_bf[X]);
	}
	board->controls_nav.rpy[YAW] = set_yaw(dir_desired_bf[X], dir_desired_bf[Y]);
	
	//board->controls_nav.rpy[ROLL] = set_roll(0.0, attitude.vel_bf[Y]);
	//board->controls_nav.rpy[PITCH] = set_pitch(0.0, attitude.vel_bf[X]);
	// board->controls_nav.rpy[YAW] = set_yaw(0.0, 0.0);
}

void high_speed_nav(double dir_desired_bf[], Quat_Attitude_t attitude)
{
	dbg_print("High speed nav\n");
	board->controls_nav.rpy[ROLL] = set_roll(dir_desired_bf[Y], attitude.vel_bf[Y]);
	board->controls_nav.rpy[PITCH] = set_pitch(dir_desired_bf[X], attitude.vel_bf[X]);
	board->controls_nav.rpy[YAW] = set_yaw(attitude.vel_bf[X], attitude.vel_bf[Y]);
	
}

void altitude_nav(double dir_desired_bf_z)
{
	int i;
	
	float err;
	
	err = dir_desired_bf_z; // - board->imu1.attitude.localPosition.pos[Z];
	
	board->controls_nav.thrust = - 0.35 * 9.81 / 16.0 + KP_ALT * err  + KI_ALT * alt_integrator; // //+ KD_ALT * board->imu1.attitude.vel[Z]
	
	alt_integrator += err; 
	
	alt_integrator = min_max_bound(alt_integrator,-1.0,1.0);
	
	//dbg_print("Thrust (x10000):");
	//dbg_print_num(board->controls_nav.thrust*10000,10);
	//dbg_print(" , err (x1000):");
	//dbg_print_num(KP_ALT *err*1000,10);
	//dbg_print(", alt_int (x1000):");
	//dbg_print_num(KI_ALT * alt_integrator*1000,10);
	//dbg_print("; dir_bf (x1000):");
	//dbg_print_num(dir_desired_bf_z*1000,10);
	//dbg_print(", localPosZ (x1000):");
	//dbg_print_num(board->imu1.attitude.localPosition.pos[Z]*1000,10);
	//dbg_print("\n");
}

float set_roll(double direction_bf_y, double vel_bf_y)
{
	float roll_set;
	
	roll_set = KP_ROLL * tanh(0.05 * (direction_bf_y - vel_bf_y));
	roll_set = min_max_bound(roll_set,MIN_ROLL_RATE,MAX_ROLL_RATE);
	
	return roll_set;
}

float set_pitch(double direction_bf_x, double vel_bf_x)
{
	float pitch_set;
	
	pitch_set = KP_PITCH * tanh(0.05 * (direction_bf_x - vel_bf_x));
	pitch_set = min_max_bound(pitch_set,MIN_PITCH_RATE,MAX_PITCH_RATE);
	
	return pitch_set;
}

float set_yaw(double value_x, double value_y)
{
	float yaw_set;
	
	if (abs(value_x) < 0.001 && abs(value_y) < 0.001 || waypoint_hold_init)
	{
		//dbg_print("wp_hold_init:");
		//dbg_print_num(waypoint_hold_init,10);
		//dbg_print(", value_x-y:");
		//dbg_print_num(value_x,10);
		//dbg_print(", ");
		//dbg_print_num(value_y,10);
		//dbg_print("\n");
		yaw_set = 0.0;
		//dbg_print("set yaw to zero\n");
	}else{
		yaw_set = KP_YAW * atan2(value_y, value_x);
	}
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

float min_max_bound(double value, double min, double max)
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