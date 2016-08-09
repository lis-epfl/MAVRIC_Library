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
 * \brief The MAVLink mission planner handler
 *
 ******************************************************************************/


#ifndef MISSION_PLANNER_HANDLER__
#define MISSION_PLANNER_HANDLER__

#include "control/mission_planner.hpp"
#include "control/waypoint.hpp"

/*
 * N.B.: Reference Frames and MAV_CMD_NAV are defined in "maveric.h"
 */

class Mission_handler
{
public:
    /**
     * \brief   Initializes the mission handler
     *
     * \details Needs to be defined in subclasses
     *
     * \return  Success
     */
    virtual bool init();

    /**
     * \brief   Checks if the handler is able to handle the request
     *  
     * \details     This must be defined in the subclasses. It should perform
     *              a check on the inputted waypoint and return true or false
     *              if this is the appropriate handler for the waypoint.
     *
     * \param   mission_planner     The mission planner class
     * \param   wpt                 The waypoint class
     *
     * \return  Can handle
     */
    virtual bool can_handle(Mission_planner& mission_planner, Waypoint& wpt) = 0;

    /**
     * \brief   Sets up this handler class for a first time initialization
     *  
     * \details     This must be defined in the subclasses. It should perform
     *              initial setup. For example, setting the hold position to
     *              the current position of the drone, that way it is not done
     *              every iteration
     *
     * \param   mission_planner     The mission planner class
     * \param   wpt                 The waypoint class
     *
     * \return  Success
     */
    virtual bool setup(Mission_planner& mission_planner, Waypoint& wpt) = 0;

    /**
     * \brief   Handles the mission every iteration
     *  
     * \details     This must be defined in the subclasses. It should perform
     *              routine checks and code that needs to be done every iteration
     *
     * \param   mission_planner     The mission planner class
     */
    virtual void handle(Mission_planner& mission_planner) = 0;

    /**
     * \brief   Checks if the handler has finished the request of the waypoint
     *  
     * \details     This must be defined in the subclasses. It should perform
     *              a check to see if the mission item has been finished. For
     *              example, check to see if we are within acceptable radius
     *              of the waypoint
     *
     * \param   mission_planner     The mission planner class
     *
     * \return  Is finished
     */
    virtual bool is_finished(Mission_planner& mission_planner) = 0;
};






#endif // MISSION_PLANNER_HANDLER__
