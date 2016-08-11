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
 * \file mission_handler_registry.hpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief The mission handler registry is responsible for registering and
 *        obtaining mission handlers based on an inputted waypoint
 *
 ******************************************************************************/


#ifndef MISSION_HANDLER_REGISTRY__
#define MISSION_HANDLER_REGISTRY__

#include "control/mission_handler.hpp"
#include "control/waypoint.hpp"

#define MAX_REGISTERED_MISSION_HANDLERS 20

/*
 * N.B.: Reference Frames and MAV_CMD_NAV are defined in "maveric.h"
 */

class Mission_handler_registry
{
public:
    Mission_handler_registry();

    /**
     * \brief   Registers the inputted handler to the array of known
     *          mission handlers. Performs a check to see if that
     *          object is already within the array.
     *
     * \param   handler     The new mission handler
     *
     * \return  Success
     */
    bool register_mission_handler(Mission_handler* handler);

    /**
     * \brief   Attempts to get the mission handler form the registry.
     *
     * \details If there is a mission handler that can take the waypoint,
     *          then the registry will return the first mission handler
     *          that can take it. If no mission handler can take the
     *          waypoint, then the mission handler will return NULL.
     *
     * \param   waypoint    The waypoint to be handled
     *
     * \return  The first acceptable mission handler or NULL
     */
    Mission_handler* get_mission_handler(Waypoint* waypoint);

private:
    /**
     * Array of registered mission handlers. Other classes will
     * register a mission handler through the function
     * Mission_planner::register_mission_handler(Mission_handler*).
     */
    Mission_handler* registered_mission_handlers_[MAX_REGISTERED_MISSION_HANDLERS];
    uint8_t registered_mission_handler_count_;                  ///< The number of mission handler in the array
};






#endif // MISSION_HANDLER_REGISTRY__
