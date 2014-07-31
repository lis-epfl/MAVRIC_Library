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
 * \file hud.c
 *
 *  This file sends the mavlink HUD message
 */


#include "hud.h"
#include "print_util.h"
#include "coord_conventions.h"
#include "mavlink_communication.h"

void hud_init(hud_structure_t* hud_structure, const position_estimator_t* pos_est, const control_command_t* controls, const ahrs_t* ahrs, const mavlink_stream_t* mavlink_stream)
{
	hud_structure->ahrs = ahrs;
	hud_structure->controls            = controls;
	hud_structure->pos_est             = pos_est;
	hud_structure->mavlink_stream      = mavlink_stream;
	
	print_util_dbg_print("HUD structure initialised.\n");
}

task_return_t hud_send_message(hud_structure_t* hud_structure) 
{
	float groundspeed = sqrt(hud_structure->pos_est->vel[0] * hud_structure->pos_est->vel[0] + hud_structure->pos_est->vel[1] * hud_structure->pos_est->vel[1]);
	float airspeed=groundspeed;

	aero_attitude_t aero_attitude;
	aero_attitude = coord_conventions_quat_to_aero(hud_structure->ahrs->qe);
	
	int16_t heading;
	if(aero_attitude.rpy[2] < 0)
	{
		heading = (int16_t)(360.0f + 180.0f * aero_attitude.rpy[2] / PI); //you want to normalize between 0 and 360°
	}
	else
	{
		heading = (int16_t)(180.0f * aero_attitude.rpy[2] / PI);
	}
	
	
	mavlink_message_t msg;	
	mavlink_msg_vfr_hud_pack(	hud_structure->mavlink_stream->sysid, 
								hud_structure->mavlink_stream->sysid,
								&msg,
								airspeed, 
								groundspeed, 
								heading, 
								(int32_t)((hud_structure->controls->thrust + 1.0f) * 50), 
								-hud_structure->pos_est->local_position.pos[2] + hud_structure->pos_est->local_position.origin.altitude, 
								-hud_structure->pos_est->vel[2]	);
	mavlink_stream_send(hud_structure->mavlink_stream, &msg);

	return TASK_RUN_SUCCESS;
}