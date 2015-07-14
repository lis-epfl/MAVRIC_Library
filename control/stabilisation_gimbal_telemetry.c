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
 * \file stabilisation_telemetry.c
 * 
 * \author Alexandre Cherpillod
 *   
 * \brief This module takes care of sending periodic telemetric messages for
 * the gimbal stabilisation module
 *
 ******************************************************************************/

#include "stabilisation_gimbal_telemetry.h"
#include "stabilisation_telemetry.h"
#include "time_keeper.h"
#include "constants.h"


void gimbal_stabilisation_telemetry_send_rpy_setpoint(const stabilisation_copter_t* stabiliser, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{	
	// Controls setpoints, from the VR goggles
	mavlink_msg_debug_vect_pack(	mavlink_stream->sysid,
									mavlink_stream->compid,
									msg,
									"z_setpoint",
									time_keeper_get_millis(),	
									stabiliser->controls->gimbal_rpy[ROLL],
									stabiliser->controls->gimbal_rpy[PITCH],
									stabiliser->controls->gimbal_rpy[YAW]);
}

void gimbal_stabilisation_telemetry_send_rpy_output(const stabiliser_t* stabiliser, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
	// Controls outputs, to the gimbal servos
	mavlink_msg_debug_vect_pack(	mavlink_stream->sysid,
									mavlink_stream->compid,
									msg,
									"z_output",
									time_keeper_get_millis(),
									stabiliser->output.gimbal_rpy[ROLL],
									stabiliser->output.gimbal_rpy[PITCH],
									stabiliser->output.gimbal_rpy[YAW]);
}

