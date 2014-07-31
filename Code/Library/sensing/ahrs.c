/** 
 * \page The MAV'RIC license
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */


/**
 * \file ahrs.c
 *
 * This file implements attitude estimation data structure
 */


#include "ahrs.h"

#include "delay.h"
#include "time_keeper.h"
#include "print_util.h"
#include "mavlink_stream.h"
#include "coord_conventions.h"

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void ahrs_init(ahrs_t* ahrs, mavlink_stream_t* mavlink_stream)
{
	// Init dependencies
	ahrs->mavlink_stream = mavlink_stream;

	// Init structure
	ahrs->qe.s = 1.0f;
	ahrs->qe.v[0] = 0.0f;
	ahrs->qe.v[1] = 0.0f;
	ahrs->qe.v[2] = 0.0f;
	
	ahrs->angular_speed[X] = 0.0f;
	ahrs->angular_speed[Y] = 0.0f;
	ahrs->angular_speed[Z] = 0.0f;
	
	ahrs->linear_acc[X] = 0.0f;
	ahrs->linear_acc[Y] = 0.0f;
	ahrs->linear_acc[Z] = 0.0f;
	
	ahrs->north_vec.s    = 0.0f;
	ahrs->north_vec.v[0] = 1.0f;
	ahrs->north_vec.v[1] = 0.0f;
	ahrs->north_vec.v[2] = 0.0f;
	
	ahrs->up_vec.s    = 0.0f;
	ahrs->up_vec.v[0] = 0.0f;
	ahrs->up_vec.v[1] = 0.0f;
	ahrs->up_vec.v[2] = -1.0f;
}

task_return_t ahrs_send_attitude(ahrs_t* ahrs)
{
	aero_attitude_t aero_attitude;
	aero_attitude = coord_conventions_quat_to_aero(ahrs->qe);

	mavlink_message_t msg;
	mavlink_msg_attitude_pack(	ahrs->mavlink_stream->sysid,
								ahrs->mavlink_stream->compid,
								&msg,
								time_keeper_get_millis(),
								aero_attitude.rpy[0],
								aero_attitude.rpy[1],
								aero_attitude.rpy[2],
								ahrs->angular_speed[0],
								ahrs->angular_speed[1],
								ahrs->angular_speed[2]);
	mavlink_stream_send(ahrs->mavlink_stream,&msg);
	
	return TASK_RUN_SUCCESS;
}

task_return_t ahrs_send_attitude_quaternion(ahrs_t* ahrs)
{
	mavlink_message_t msg;
	mavlink_msg_attitude_quaternion_pack(	ahrs->mavlink_stream->sysid,
											ahrs->mavlink_stream->compid,
											&msg,
											time_keeper_get_millis(),
											ahrs->qe.s,
											ahrs->qe.v[0],
											ahrs->qe.v[1],
											ahrs->qe.v[2],
											ahrs->angular_speed[0],
											ahrs->angular_speed[1],
											ahrs->angular_speed[2]	);
	mavlink_stream_send(ahrs->mavlink_stream,&msg);
	
	return TASK_RUN_SUCCESS;
}