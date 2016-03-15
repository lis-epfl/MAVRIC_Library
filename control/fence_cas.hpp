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
 * \file fence_avoiding.hpp
 *
 * \author MAV'RIC Team
 * \author Cyril Stuber
 *
 * \brief This module takes care of simulating a fence and avoiding it.
 *
 ******************************************************************************/


#ifndef FENCE_CAS_H_
#define FENCE_CAS_H_


//chechk if thoses are usefull

#include "communication/mavlink_waypoint_handler.hpp"
#include "sensing/position_estimation.hpp"

extern "C"
{
#include "control/control_command.h"
}

/**
 * \brief Fence configuration
 */
/*
typedef struct
{
	mavlink_waypoint_handler_t&   		waypoint_t;	///< Waypoint handler
	uint8_t 							fence_id;	///< Store thee the fence index
	uint8_t 							point_index[MAX_WAYPOINTS]; ///< List of the points in the fence
} fence_t;
*/

class Fence_CAS
{
public:
	Fence_CAS(void);
	~Fence_CAS(void);
	bool update(void);
	void add_fence(void);
	void del_fence(uint8_t fence_id);

private:

	const mavlink_waypoint_handler_t*   waypoint_handler;					///< Waypoints handler
	uint8_t								fence_id;					///< Id of the fence
	uint8_t								point_index[MAX_WAYPOINTS];	///< Fence Id for each of the waypoints
	const position_estimation_t*        pos_est;                    ///< Estimated position and speed (input)
	velocity_command_t*                 velocity_command;           ///< Velocity command (output)
};

#endif /*FENCE_CAS_H_*/
