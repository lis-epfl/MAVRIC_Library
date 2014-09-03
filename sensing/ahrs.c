/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*******************************************************************************
 * \file ahrs.c
 * 
 * \author MAV'RIC Team
 * \author Gregoire Heitz
 *   
 * \brief This file implements data structure for attitude estimate
 *
 ******************************************************************************/
 

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