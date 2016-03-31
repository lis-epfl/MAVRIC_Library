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
 * \file mavlink_waypoint_handler.h
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *
 * \brief The MAVLink waypoint handler
 *
 ******************************************************************************/


#ifndef MAVLINK_WAYPOINT_HANDLER_SWARM__
#define MAVLINK_WAYPOINT_HANDLER_SWARM__

#include "communication/mavlink_waypoint_handler.hpp"

/*
 * N.B.: Reference Frames and MAV_CMD_NAV are defined in "maveric.h"
 */


class Mavlink_waypoint_handler_swarm : Mavlink_waypoint_handler
{
    Mavlink_waypoint_handler_swarm(Position_estimation& position_estimation,
                           Navigation& navigation,
                           const ahrs_t& ahrs,
                           State& state,
                           const manual_control_t& manual_control,
                           Mavlink_message_handler& message_handler,
                           const Mavlink_stream& mavlink_stream);
private:
    /**
     * \brief   Sets a circle scenario, where two waypoints are set at opposite side of the circle
     *
     * \param   packet                  The pointer to the structure of the MAVLink command message long
     */
    void set_circle_scenario(mavlink_command_long_t* packet);

    /**
     * \brief   Sets a circle scenario, where n waypoints are set at random position on a circle
     *
     * \param   packet                  The pointer to the structure of the MAVLink command message long
     */
    void set_circle_uniform_scenario(mavlink_command_long_t* packet);

    /**
     * \brief   Sets a stream scenario, where two flows of MAVs go in opposite ways
     *
     * \param   packet                  The pointer to the structure of the MAVLink command message long
     */
    void set_stream_scenario(mavlink_command_long_t* packet);

    /**
     * \brief   Sets a swarm scenario, where two flocks (grouping) of MAVs go in opposite ways
     *
     * \param   packet                  The pointer to the structure of the MAVLink command message long
     */
    void set_swarm_scenario(mavlink_command_long_t* packet);


    /**
     * \brief   Sets a scenario for multiple MAV case
     *
     * \param   waypoint_handler        The pointer to the structure of the MAVLink waypoint handler
     * \param   packet                  The pointer to the structure of the MAVLink command message long
     *
     * \return  mav_result_t            The result of the scenario setting up
     */
    static mav_result_t set_scenario(Mavlink_waypoint_handler_swarm* waypoint_handler, mavlink_command_long_t* packet);


};


#endif // MAVLINK_WAYPOINT_HANDLER_SWARM__