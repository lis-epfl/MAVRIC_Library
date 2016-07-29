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
 * \file mission_planner_handler.hpp
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
#include "communication/waypoint.hpp"
#include "sensing/ins.hpp"

/*
 * N.B.: Reference Frames and MAV_CMD_NAV are defined in "maveric.h"
 */

class Mission_planner_handler
{
public:
    /**
     * \brief   Constructor
     *
     * \param   ins     The reference to the ins class
     */
    Mission_planner_handler(const INS& ins);

    /**
     * \brief   The handler for the mission planner. This is called by the state
     * machine in Mission_planner. It should check which subprocess (e.g. land,
     * emergency land, land on tag, etc.) if necessary and handler the request.
     *
     * \param   mission_planner     The reference to the misison planner that is
     * handling the request.
     */
    virtual void handle(Mission_planner& mission_planner) = 0;

    virtual bool init() = 0;

    /**
     * \brief   Resets the hold position waypoint
     *
     * If the drone tries to access the hold position waypoint and it has not
     * been set, the waypoint will be set to the current position estimation.
     */
    static void reset_hold_waypoint();

    /**
     * \brief   Gets the flag stating if the hold waypoint has been set
     *
     * \return  hold_waypoint_set_
     */
    static bool hold_waypoint_set();

    /**
     * \brief   Sets the hold waypoint to the hold position
     *
     * \param   hold_position   The new desired hold position
     */
    static void set_hold_waypoint(const local_position_t hold_position);

    /**
     * \brief   Sets the hold waypoint to the hold position
     *
     * \param   hold_position   The new desired hold position
     * \param   heading         The desired heading of the waypoint
     */
    static void set_hold_waypoint(const local_position_t hold_position, float heading);

    /**
     * \brief   Sets the hold waypoint to the inputted waypoint
     *
     * \param   wpt     The new waypoint
     */
    static void set_hold_waypoint(const Waypoint wpt);

protected:
    /**
     * \brief   Gets the hold waypoint
     *
     * If no waypoint has been assigned to the hold waypoint, this function will
     * set the current position estimation to the waypoint and return it.
     */
    Waypoint& hold_waypoint();

    const INS& ins_;                            ///< The ins reference

private:

    static bool hold_waypoint_set_;             ///< Flag stating if the hold waypoint has been set. If it hasn't, set to current position estimation when first called
    static Waypoint hold_waypoint_;             ///< The hold waypoint, used by various child classes to set the hold position
};






#endif // MISSION_PLANNER_HANDLER__
