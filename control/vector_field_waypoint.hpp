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
 * \file vector_field_waypoint.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief Vector field navigation using repulsors and attractors set through GPS waypoints
 *
 ******************************************************************************/


#ifndef VECTOR_FIELD_WAYPOINT_HPP_
#define VECTOR_FIELD_WAYPOINT_HPP_

#include "communication/mavlink_waypoint_handler.hpp"
#include "sensing/position_estimation.hpp"

extern "C"
{
#include "control/control_command.h"
}

/**
 * \brief Vector field navigation
 */
typedef struct
{
    const Mavlink_waypoint_handler*   waypoint_handler;         ///< Waypoint list (input)
    const INS*                        ins;                      ///< Estimated position and speed (input)
    velocity_command_t*               velocity_command;         ///< Velocity command (output)
} vector_field_waypoint_t;


/**
 * \brief Attitude controller configuration
 */
typedef struct
{
} vector_field_waypoint_conf_t;


/**
 * \brief                       Initialises the attitude controller structure
 *
 * \param   vector_field        Pointer to data structure
 * \param   waypoint_handler    Pointer to waypoint list (input)
 * \param   ins                 Pointer to the Inertial Navigation System (input)
 * \param   velocity_command    Pointer to velocity command (output)
 * \param   config              Pointer to configuration
 */
bool vector_field_waypoint_init(vector_field_waypoint_t* vector_field, const Mavlink_waypoint_handler* waypoint_handler, const INS* ins, velocity_command_t* velocity_command, const vector_field_waypoint_conf_t* config);


/**
 * \brief                   Main update function
 *
 * \param   vector_field    Pointer to data structure
 */
bool vector_field_waypoint_update(vector_field_waypoint_t* vector_field);


#endif /* VECTOR_FIELD_WAYPOINT_HPP_ */
