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
 * \file mission_handler_on_ground.hpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief The MAVLink mission planner handler for the on ground state
 *
 ******************************************************************************/


#ifndef MISSION_HANDLER_ON_GROUND__
#define MISSION_HANDLER_ON_GROUND__

#include "control/controller.hpp"
#include "mission/mission_handler.hpp"

/*
 * N.B.: Reference Frames and MAV_CMD_NAV are defined in "maveric.h"
 */

class Mission_handler_on_ground : public Mission_handler
{
public:


    /**
     * \brief   Initialize the on ground mission planner handler
     *
     * \param   rate_controller     The reference to the attitude controls
     */
     Mission_handler_on_ground(Controller<rate_command_t>& rate_controller);


    /**
     * \brief   Checks if the waypoint is on the ground
     *
     * \details     DOES NOT CURRENTLY CHECK IF WE ARE ON GROUND
     *
     * \param   wpt                 The waypoint class
     *
     * \return  Can handle
     */
    virtual bool can_handle(const Waypoint& wpt) const;

    /**
     * \brief   Does nothing
     *
     * \details     Does nothing
     *
     * \param   wpt                 The waypoint class
     *
     * \return  True
     */
    virtual bool setup(const Waypoint& wpt);

    /**
     * \brief   Returns MISSION_IN_PROGRESS
     *
     * \details     Sets thrust to low and returns MISSION_IN_PROGRESS
     *
     * \return  MISSION_IN_PROGRESS
     */
    virtual Mission_handler::update_status_t update();

    /**
     * \brief   Returns that the mission state is in STANDBY
     *
     * \return  Mission handler's mission state
     */
    virtual Mission_planner::internal_state_t handler_mission_state() const;

protected:
    Controller<rate_command_t>& rate_controller_;                    ///< The reference to the rate controller
};







#endif // MISSION_HANDLER_ON_GROUND__
