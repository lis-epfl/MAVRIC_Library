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

void hud_init(hud_structure_t *hud_structure, position_estimator_t *pos_est, Control_Command_t *controls, ahrs_t *attitude_estimation)
{
	hud_structure->attitude_estimation = attitude_estimation;
	hud_structure->controls = controls;
	hud_structure->pos_est = pos_est;
	
	print_util_dbg_print("HUD structure initialised.\n");
}

task_return_t hud_send_message(hud_structure_t* hud_structure) 
{
	float groundspeed = sqrt(hud_structure->pos_est->vel[0] * hud_structure->pos_est->vel[0] + hud_structure->pos_est->vel[1] * hud_structure->pos_est->vel[1]);
	float airspeed=groundspeed;

	Aero_Attitude_t aero_attitude;
	aero_attitude = coord_conventions_quat_to_aero(hud_structure->attitude_estimation->qe);
	
	int16_t heading;
	if(aero_attitude.rpy[2] < 0)
	{
		heading = (int16_t)(360.0f + 180.0f * aero_attitude.rpy[2] / PI); //you want to normalize between 0 and 360°
	}
	else
	{
		heading = (int16_t)(180.0f * aero_attitude.rpy[2] / PI);
	}
	
	
	
	// mavlink_msg_vfr_hud_send(mavlink_channel_t chan, float airspeed, float groundspeed, int16_t heading, uint16_t throttle, float alt, float climb)
	mavlink_msg_vfr_hud_send(	MAVLINK_COMM_0, 
								airspeed, 
								groundspeed, 
								heading, 
								(int32_t)((hud_structure->controls->thrust + 1.0f) * 50), 
								-hud_structure->pos_est->localPosition.pos[2] + hud_structure->pos_est->localPosition.origin.altitude, 
								-hud_structure->pos_est->vel[2]	);
	
	return TASK_RUN_SUCCESS;
}