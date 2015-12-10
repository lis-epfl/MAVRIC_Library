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
 * \file dubin.h
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *   
 * \brief Vector field navigation using Dubin's path
 *
 ******************************************************************************/

#ifndef DUBIN_H_
#define DUBIN_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "vectors.h"
#include <stdint.h>
#include <stdbool.h>
#include "maths.h"
#include "mav_modes.h"
#include "constants.h"

typedef struct
{
	float circle_center_1[3];
	float tangent_point_1[3];
	int8_t sense_1;
	float circle_center_2[3];
	float tangent_point_2[3];
	float line_direction[3];
	float length;
	float old_radius;
}dubin_t;

void dubin_line(float tvel[3], const float line_dir[3], const float line_origin[3], const float pos[3], const float speed, const float one_over_scaling);

void dubin_circle(float tvel[3], const float circle[3], const float radius, const float pos[3], const float speed, const float one_over_scaling);

/**
 * \brief 		Creating Dubin's path between two waypoints and a sense of rotation
 * 
 * \details 	Computes a 3D velocity vector guiding the MAV around a circular waypoint
 * 				 
 * \param 		rel_pos 		Current relative position of the MAV to the waypoint (input)
 * \param 		attractiveness	Weight given to this object (input)
 * \param 		attractiveness2	Weight given to this object (input)
 * \param 		cruise_speed	Nominal speed around the circle in m/s (input)
 * \param 		radius			Radius of the circle to follow (in m)
 * \param 		vector			Velocity command vector (output)
 *
 * \return		Return true if everything is ok, false if there is a computational problem.
 */
dubin_t dubin_2d(const float wp1[3], const float wp2[3], const float d1[3], const float d2[3], float sense_2);

#ifdef __cplusplus
}
#endif

#endif /* DUBIN_H_ */