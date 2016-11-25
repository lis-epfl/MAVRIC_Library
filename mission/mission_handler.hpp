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
 * \file mission_handler.hpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief Interface for mission planner handler
 *
 ******************************************************************************/


#ifndef MISSION_HANDLER_HPP_
#define MISSION_HANDLER_HPP_

#include "mission/mission_planner.hpp"
#include "mission/waypoint.hpp"
#include "control/flight_controller.hpp"

/*
 * \brief   Interface for mission planner handler
 *
 * \detail  This mission handler interface. Outine functions that child classes must
 *          implement in order to have the drone achieve the desired mission command.
 */
class Mission_handler: public Flight_command_source
{
public:
    enum update_status_t
    {
        MISSION_FAILED=-1,
        MISSION_IN_PROGRESS=0,
        MISSION_FINISHED=1
    };


    /**
     * \brief   Checks if the handler is able to handle the request
     *
     * \details     This must be defined in the subclasses. It should perform
     *              a check on the inputted waypoint and return true or false
     *              if this is the appropriate handler for the waypoint.
     *
     * \param   wpt                 The waypoint class
     *
     * \return  Can handle
     */
    virtual bool can_handle(const Waypoint& wpt) const = 0;


    /**
     * \brief   Sets up this handler class for a first time initialization
     *
     * \details     This must be defined in the subclasses. It should perform
     *              initial setup. For example, setting the hold position to
     *              the current position of the drone, that way it is not done
     *              every iteration.
     *
     * \param   wpt                 The waypoint class
     *
     * \return  Success
     */
    virtual bool setup(const Waypoint& wpt) = 0;


    /**
     * \brief   Handles the mission every iteration
     *
     * \details     This must be defined in the subclasses. It should perform
     *              routine checks and code that needs to be done every iteration
     *              The effective goal of the handle function is to set some
     *              command that will have the drone achieve the mission item.
     *
     * \return  Update status code
     */
    virtual update_status_t update() = 0;


    /**
     * \brief   Gets the mission state of this handler
     *
     * \details     All mission handlers are required to suggest a mission state.
     *              The choices can be one of the follow:
     *                  STANDBY (drone is on ground and standing by)
     *                  PREMISSION (takeoff)
     *                  MISSION (executing mission waypoint)
     *                  POSTMISSION (landing)
     *                  PAUSED (should not be selected)
     *              Child classes should implement this function to return one
     *              of these states.
     *
     * \return  Mission handler's mission state
     */
    virtual Mission_planner::internal_state_t handler_mission_state() const = 0;
};

#endif // MISSION_HANDLER_HPP_
