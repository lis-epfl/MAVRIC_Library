/*******************************************************************************
 * Copyright (c) 2009-2016, MAV'RIC Development Team
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
 * \file fence.hpp
 *
 * \author MAV'RIC Team
 * \author Cyril Stuber
 *
 * \brief This module takes care of simulating a fence and avoiding it.
 *
 ******************************************************************************/


#ifndef FENCE_H_
#define FENCE_H_

#include "communication/mavlink_waypoint_handler.hpp"


extern "C"
{
#include "control/control_command.h"
#include "control/navigation.hpp"
}


class Fence
{
public:
	Fence(mavlink_waypoint_handler_t* waypoint_handler);
	~Fence(void);
	void add_waypoint(waypoint_struct_t* new_waypoint);
	void del_waypoint(void);
	void set_h_max(float new_h);
	float get_h_max(void);
	void set_h_min(float new_h);
	float get_h_min(void);
	uint8_t get_fence_id(void);
private:
	//waypoint params: p1,p2,p3,p4,x,y,z
	//attribution 			h_min,h_max,fence_id;
	waypoint_struct_t 	fence_points[MAX_WAYPOINTS];
	uint8_t 			fence_id=1;
	float				h_max=20; //[m]
	float				h_min=5; //[m]
	uint8_t				nb_wall=0;
	uint8_t				nb_waypoint=0; ///< number of waypoints in the fence

};

#endif /*FENCE_H_*/
