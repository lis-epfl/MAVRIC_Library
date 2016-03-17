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
 * \file ahrs_telemetry.c
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *   
 * \brief This module takes care of sending periodic telemetric messages for
 * the ahrs module
 *
 ******************************************************************************/


#include "ahrs_telemetry.h"
#include "time_keeper.h"
#include "coord_conventions.h"

void ahrs_telemetry_send_attitude(const ahrs_t* ahrs, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
	aero_attitude_t aero_attitude;
	aero_attitude = coord_conventions_quat_to_aero(ahrs->qe);

	mavlink_msg_attitude_pack(	mavlink_stream->sysid,
								mavlink_stream->compid,
								msg,
								time_keeper_get_millis(),
								aero_attitude.rpy[0],
								aero_attitude.rpy[1],
								aero_attitude.rpy[2],
								ahrs->angular_speed[0],
								ahrs->angular_speed[1],
								ahrs->angular_speed[2]);
}

void ahrs_telemetry_send_attitude_quaternion(const ahrs_t* ahrs, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
	mavlink_msg_attitude_quaternion_pack(	mavlink_stream->sysid,
											mavlink_stream->compid,
											msg,
											time_keeper_get_millis(),
											ahrs->qe.s,
											ahrs->qe.v[0],
											ahrs->qe.v[1],
											ahrs->qe.v[2],
											ahrs->angular_speed[0],
											ahrs->angular_speed[1],
											ahrs->angular_speed[2]	);
}

void ahrs_telemetry_send_attitude_quaternion_cov(const ahrs_t* ahrs, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
	float qe[4];
	qe[0] = ahrs->qe.s;
	qe[1] = ahrs->qe.v[0];
	qe[2] = ahrs->qe.v[1];
	qe[3] = ahrs->qe.v[2];

	float cov[9];
	cov[0] = 0.0f;
	cov[1] = 0.0f;
	cov[2] = 0.0f;
	cov[3] = 0.0f;
	cov[4] = 0.0f;
	cov[5] = 0.0f;
	cov[6] = 0.0f;
	cov[7] = 0.0f;
	cov[8] = 0.0f;
	mavlink_msg_attitude_quaternion_cov_pack(	mavlink_stream->sysid,
												mavlink_stream->compid,
												msg,
												time_keeper_get_millis(),
												qe,
												ahrs->angular_speed[0],
												ahrs->angular_speed[1],
												ahrs->angular_speed[2],
												cov);
}

void ahrs_telemetry_send_P_diag(const ahrs_t* ahrs, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
	mavlink_msg_debug_vect_pack(	mavlink_stream->sysid,
									mavlink_stream->compid,
									msg,"P_diag012",
									time_keeper_get_micros(),
									ahrs->P_vect[0],
									ahrs->P_vect[8],
									ahrs->P_vect[16]);
	mavlink_stream_send(mavlink_stream,msg);

	mavlink_msg_debug_vect_pack(	mavlink_stream->sysid,
									mavlink_stream->compid,
									msg,"P_diag345",
									time_keeper_get_micros(),
									ahrs->P_vect[24],
									ahrs->P_vect[32],
									ahrs->P_vect[40]);
	mavlink_stream_send(mavlink_stream,msg);

	mavlink_msg_named_value_float_pack(	mavlink_stream->sysid,
										mavlink_stream->compid,
										msg,
										time_keeper_get_millis(),
										"P_diag6",
										ahrs->P_vect[48]);

	/*mavlink_msg_debug_vect_pack(	mavlink_stream->sysid,
									mavlink_stream->compid,
									msg,"P_diag10",
									time_keeper_get_micros(),
									ahrs->P_vect[0],
									ahrs->P_vect[1],
									ahrs->P_vect[2]);
	mavlink_stream_send(mavlink_stream,msg);

	mavlink_msg_debug_vect_pack(	mavlink_stream->sysid,
									mavlink_stream->compid,
									msg,"P_diag11",
									time_keeper_get_micros(),
									ahrs->P_vect[3],
									ahrs->P_vect[4],
									ahrs->P_vect[5]);
	mavlink_stream_send(mavlink_stream, msg);

	mavlink_msg_named_value_float_pack(	mavlink_stream->sysid,
										mavlink_stream->compid,
										msg,
										time_keeper_get_millis(),
										"P_diag12",
										ahrs->P_vect[6]);
	mavlink_stream_send(mavlink_stream, msg);

	mavlink_msg_debug_vect_pack(	mavlink_stream->sysid,
									mavlink_stream->compid,
									msg,"P_diag20",
									time_keeper_get_micros(),
									ahrs->P_vect[7],
									ahrs->P_vect[8],
									ahrs->P_vect[9]);
	mavlink_stream_send(mavlink_stream,msg);

	mavlink_msg_debug_vect_pack(	mavlink_stream->sysid,
									mavlink_stream->compid,
									msg,"P_diag21",
									time_keeper_get_micros(),
									ahrs->P_vect[10],
									ahrs->P_vect[11],
									ahrs->P_vect[12]);
	mavlink_stream_send(mavlink_stream, msg);

	mavlink_msg_named_value_float_pack(	mavlink_stream->sysid,
										mavlink_stream->compid,
										msg,
										time_keeper_get_millis(),
										"P_diag22",
										ahrs->P_vect[13]);
	mavlink_stream_send(mavlink_stream, msg);

	mavlink_msg_debug_vect_pack(	mavlink_stream->sysid,
									mavlink_stream->compid,
									msg,"P_diag30",
									time_keeper_get_micros(),
									ahrs->P_vect[14],
									ahrs->P_vect[15],
									ahrs->P_vect[16]);
	mavlink_stream_send(mavlink_stream,msg);

	mavlink_msg_debug_vect_pack(	mavlink_stream->sysid,
									mavlink_stream->compid,
									msg,"P_diag31",
									time_keeper_get_micros(),
									ahrs->P_vect[17],
									ahrs->P_vect[18],
									ahrs->P_vect[19]);
	mavlink_stream_send(mavlink_stream, msg);

	mavlink_msg_named_value_float_pack(	mavlink_stream->sysid,
										mavlink_stream->compid,
										msg,
										time_keeper_get_millis(),
										"P_diag32",
										ahrs->P_vect[20]);
	mavlink_stream_send(mavlink_stream, msg);

	mavlink_msg_debug_vect_pack(	mavlink_stream->sysid,
									mavlink_stream->compid,
									msg,"P_diag40",
									time_keeper_get_micros(),
									ahrs->P_vect[21],
									ahrs->P_vect[22],
									ahrs->P_vect[23]);
	mavlink_stream_send(mavlink_stream,msg);

	mavlink_msg_debug_vect_pack(	mavlink_stream->sysid,
									mavlink_stream->compid,
									msg,"P_diag41",
									time_keeper_get_micros(),
									ahrs->P_vect[24],
									ahrs->P_vect[25],
									ahrs->P_vect[26]);
	mavlink_stream_send(mavlink_stream, msg);

	mavlink_msg_named_value_float_pack(	mavlink_stream->sysid,
										mavlink_stream->compid,
										msg,
										time_keeper_get_millis(),
										"P_diag42",
										ahrs->P_vect[27]);
	mavlink_stream_send(mavlink_stream, msg);

	mavlink_msg_debug_vect_pack(	mavlink_stream->sysid,
									mavlink_stream->compid,
									msg,"P_diag50",
									time_keeper_get_micros(),
									ahrs->P_vect[28],
									ahrs->P_vect[29],
									ahrs->P_vect[30]);
	mavlink_stream_send(mavlink_stream,msg);

	mavlink_msg_debug_vect_pack(	mavlink_stream->sysid,
									mavlink_stream->compid,
									msg,"P_diag51",
									time_keeper_get_micros(),
									ahrs->P_vect[31],
									ahrs->P_vect[32],
									ahrs->P_vect[33]);
	mavlink_stream_send(mavlink_stream, msg);

	mavlink_msg_named_value_float_pack(	mavlink_stream->sysid,
										mavlink_stream->compid,
										msg,
										time_keeper_get_millis(),
										"P_diag52",
										ahrs->P_vect[34]);
	mavlink_stream_send(mavlink_stream, msg);

	mavlink_msg_debug_vect_pack(	mavlink_stream->sysid,
									mavlink_stream->compid,
									msg,"P_diag60",
									time_keeper_get_micros(),
									ahrs->P_vect[35],
									ahrs->P_vect[36],
									ahrs->P_vect[37]);
	mavlink_stream_send(mavlink_stream,msg);

	mavlink_msg_debug_vect_pack(	mavlink_stream->sysid,
									mavlink_stream->compid,
									msg,"P_diag61",
									time_keeper_get_micros(),
									ahrs->P_vect[38],
									ahrs->P_vect[39],
									ahrs->P_vect[40]);
	mavlink_stream_send(mavlink_stream, msg);

	mavlink_msg_named_value_float_pack(	mavlink_stream->sysid,
										mavlink_stream->compid,
										msg,
										time_keeper_get_millis(),
										"P_diag62",
										ahrs->P_vect[41]);
	mavlink_stream_send(mavlink_stream, msg);

	mavlink_msg_debug_vect_pack(	mavlink_stream->sysid,
									mavlink_stream->compid,
									msg,"P_diag70",
									time_keeper_get_micros(),
									ahrs->P_vect[42],
									ahrs->P_vect[43],
									ahrs->P_vect[44]);
	mavlink_stream_send(mavlink_stream,msg);

	mavlink_msg_debug_vect_pack(	mavlink_stream->sysid,
									mavlink_stream->compid,
									msg,"P_diag71",
									time_keeper_get_micros(),
									ahrs->P_vect[45],
									ahrs->P_vect[46],
									ahrs->P_vect[47]);
	mavlink_stream_send(mavlink_stream, msg);

	mavlink_msg_named_value_float_pack(	mavlink_stream->sysid,
										mavlink_stream->compid,
										msg,
										time_keeper_get_millis(),
										"P_diag72",
										ahrs->P_vect[48]);*/

}