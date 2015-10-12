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
 * \file curvace_telemetry.c
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur   
 *
 * \brief Telemetry for the cylindrical curvace
 *
 ******************************************************************************/

#include "curvace_telemetry.h"
#include "time_keeper.h"

void curvace_telemetry_send(const curvace_t* curvace, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
	static uint8_t sensor_id = 0;
	
	int16_t of_azimuth[CURVACE_NB_OF];
	int16_t of_elevation[CURVACE_NB_OF];
	int16_t azimuth[CURVACE_NB_OF];
	int16_t elevation[CURVACE_NB_OF];

	uint8_t info[CURVACE_NB_OF];
	uint8_t offset;

	uint8_t nb_of_per_message = 18;


	// fill data in 16bits
	for (uint8_t i = 0; i < CURVACE_NB_OF; ++i)
	{
		of_azimuth[i] 	= 1000 * curvace->of.all[i].x;
		of_elevation[i] = 1000 * curvace->of.all[i].y;
		azimuth[i] 		= 1000 * curvace->roi_coord.all[i].azimuth;
		elevation[i] 	= 1000 * curvace->roi_coord.all[i].elevation;
		info[i] 		= 0;
	}
	
	// Send in 6 chunks
	offset = sensor_id * nb_of_per_message;
	mavlink_msg_spherical_optic_flow_pack(	mavlink_stream->sysid,
											mavlink_stream->compid,
											msg,
											time_keeper_get_millis(),
											sensor_id,
											4,
											nb_of_per_message,
											0,
											of_azimuth   + offset,
											of_elevation + offset,
											azimuth 	 + offset,
											elevation 	 + offset,
											info 		 + offset);
	
	// Increment sensor id
	sensor_id = ( sensor_id + 1 ) % 6;
}