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
 * \file acoustic_recieve.h
 * 
 * \author MAV'RIC Team
 * \author Meysam Basiri
 *   
 * \brief Acoustic receive
 *
 ******************************************************************************/

#ifndef ACOUSTIC_RECIEVE_H_
#define ACOUSTIC_RECIEVE_H_


#include <stdint.h>
#include <stdbool.h>
#include "streams.h"

#define STORE_SIZE			4		// number of az el values stored for relaiblity test
#define RELIABILITY_ARC		0.25f	// the threshold to consider ameasurement is relaible (compared with previous 3 measurements)
#define WAIT_LIMIT			6       //wait for WAITLIMIT*ACOUSTIC_TASK_ITERATION ms to recieve the new measurement, else reset reliability
#define MAX_DETECTION_RANGE 100

typedef struct  
{
	int16_t	Azimuth;
	int16_t Elevation;
	bool	NewData;
	bool	ReliabeData;
	float	ReliabeAz;
	float	ReliabeEl;
	float	wpt[2];
}audio_Data_type;

void turn_on_siren(byte_stream_t *out_stream);
void turn_off_siren(byte_stream_t *out_stream);
void process_acoustics(void);
void recieve_acoustic(void);
void check_reliability(void);
void set_speed_command_acoustic(float rel_pos[], float dist2wpSqr);
void set_waypoint_command_acoustic(void);

#endif