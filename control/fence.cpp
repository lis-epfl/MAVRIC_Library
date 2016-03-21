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
 * \file fence.cpp
 *
 * \author MAV'RIC Team
 * \author Cyril Stuber
 *
 * \brief This module takes care of simulating a fence and avoiding it.
 *
 ******************************************************************************/


#include "fence.hpp"

extern "C"
{
#include "util/print_util.h"
#include "hal/common/time_keeper.hpp"
#include "util/coord_conventions.h"
#include "util/constants.h"
#include "util/vectors.h"
}

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------
Fence::Fence(mavlink_waypoint_handler_t* waypoint_handler):
		fence_points(waypoint_handler->fence_list)
{

}
Fence::~Fence(void)
{
	//destructeur
}
void Fence::add_waypoint(waypoint_struct_t* new_waypoint)
{
	if(nb_waypoint<MAX_WAYPOINTS)
	{
		this->fence_points[this->nb_waypoint]= *new_waypoint;
		this->nb_waypoint++;
		//print_util_dbg_print_num(waypoint_handler->number_of_waypoints - waypoint_handler->num_waypoint_onboard, 10);
		print_util_dbg_print("NEW WAYPOINT ADDED TO FENCE\r\n");
	}

}

void Fence::del_waypoint(void)
{
	if(nb_waypoint > 0)
	{
		//this->fence_points[this->nb_waypoint]= nullptr;
		this->nb_waypoint--;
	}
}
void Fence::set_h_max(float new_h)
{
	this->h_max=new_h;
}
float Fence::get_h_max(void)
{
	return this->h_max;
}
void Fence::set_h_min(float new_h)
{
	this->h_min=new_h;
}
float Fence::get_h_min(void)
{
	return this->h_min;
}
uint8_t Fence::get_fence_id(void)
{
	return this->fence_id;
}

// to use:

//position_estimation_t
/*float vel_bf[3];                        ///< 3D velocity in body frame
    float vel[3];                           ///< 3D velocity in global frame

    float last_alt;                         ///< Value of the last altitude estimation
    float last_vel[3];                      ///< Last 3D velocity

    local_position_t local_position;        ///< Local position
    local_position_t last_gps_pos;          ///< Coordinates of the last GPS position

    bool fence_set; */
