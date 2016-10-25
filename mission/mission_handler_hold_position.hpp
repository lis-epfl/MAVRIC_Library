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
 * \file mission_handler_hold_position.hpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief The MAVLink mission planner handler for the hold position state
 *
 ******************************************************************************/


#ifndef MISSION_HANDLER_HOLD_POSITION__
#define MISSION_HANDLER_HOLD_POSITION__

#include "mission/mission_handler.hpp"
#include "navigation/navigation.hpp"

/*
 * The handler class takes in a template parameter that allows control inputs.
 */
class Mission_handler_hold_position : public Mission_handler
{
public:


    /**
     * \brief   Initialize the hold position mission planner handler
     *
     * \param   controller              The reference to the controller
     * \param   ins                     The reference to the ins
     */
     Mission_handler_hold_position( Navigation& controller,
                                    const INS& ins);

    /**
     * \brief   Checks if the waypoint is a hold position waypoint
     *
     * \details     Checks if this is a:
                        MAV_CMD_NAV_LOITER_UNLIM
                        MAV_CMD_NAV_LOITER_TIME
                        MAV_CMD_NAV_LOITER_TO_ALT
                        MAV_CMD_OVERRIDE_GOTO if param1 == MAV_GOTO_DO_HOLD
     *
     * \param   wpt                 The waypoint class
     *
     * \return  Can handle
     */
    virtual bool can_handle(const Waypoint& wpt) const;

    /**
     * \brief   Sets up this handler class for a first time initialization
     *
     * \details     Stores the waypoint reference and records the starting
                    time
     *
     * \param   wpt                 The waypoint class
     *
     * \return  Success
     */
    virtual bool setup(const Waypoint& wpt);

    /**
     * \brief   Handles the mission every iteration
     *
     * \details     Sets the waypoint goal to the setup waypoint. Returns MISSION_IN_PROGRESS
     *              if the drone is still holding position, MISSION_FINISHED if it should move
     *              to the next waypoint, and MISSION_FAILED if the drone cannot hold position
     *
     * \return  Status code
     */
    virtual Mission_handler::update_status_t update();

    /**
     * \brief   Returns that the mission state is in MISSION
     *
     * \return  Mission handler's mission state
     */
    virtual Mission_planner::internal_state_t handler_mission_state() const;

protected:
    Waypoint waypoint_;                 ///< Pointer to the inputted waypoint
    uint64_t start_time_;               ///< The start time of the waypoint hold
    bool within_radius_;                ///< Flag stating if we are within the radius

    Navigation& controller_;                     ///< The reference to the controller
    const INS& ins_;                    ///< The reference to the ins structure

    /**
     * \brief   Function to set the controller specific command
     *
     * \return  Controller accepted input
     */
    virtual bool set_control_command();
};

#endif // MISSION_HANDLER_HOLD_POSITION__
