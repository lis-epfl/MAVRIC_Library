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
 * \file mission_handler_takeoff.hpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 * \author Julien Lecoeur
 *
 * \brief The mission handler for the takeoff
 *
 ******************************************************************************/


#ifndef MISSION_HANDLER_TAKEOFF_HPP_
#define MISSION_HANDLER_TAKEOFF_HPP_

#include "status/state.hpp"
#include "mission/mission_handler.hpp"

/*
 * \brief The mission handler for the takeoff
 */
class Mission_handler_takeoff : public Mission_handler
{
public:


    /**
     * \brief   Initialize the takeoff mission planner handler
     *
     * \param   ins                     The reference to the ins
     * \param   state                   The reference to the state class
     */
     Mission_handler_takeoff(const INS& ins, State& state);

    /**
     * \brief   Checks if the waypoint is a takeoff waypoint
     *
     * \details     Checks if the inputted waypoint is a:
     *                  MAV_CMD_NAV_TAKEOFF
     *
     * \param   wpt                 The waypoint class
     *
     * \return  Can handle
     */
    virtual bool can_handle(const Waypoint& wpt) const;

    /**
     * \brief   Sets up this handler class for a first time initialization
     *
     * \details     Records the waypoint reference and sets the mav mode
     *
     * \param   wpt                 The waypoint class
     *
     * \return  Success
     */
    virtual bool setup(const Waypoint& wpt);

    /**
     * \brief   Handles the mission every iteration
     *
     * \details     Sets the goal and determines the handler status. The status
     *              is: MISSION_IN_PROGRESS for takeoff in process, MISSION_FINISHED for takeoff complete, and
     *              MISSION_FAILED for control impossible
     *
     * \return  Status code
     */
    virtual Mission_handler::update_status_t update();


    /**
     * \brief   Provides control commands to the flight controller
     *
     * \return  success
     */
    virtual bool write_flight_command(Flight_controller& flight_controller) const;


    /**
     * \brief   Returns that the mission state is in PREMISSION
     *
     * \return  Mission handler's mission state
     */
    virtual Mission_planner::internal_state_t handler_mission_state() const;

protected:
    const INS& ins_;                ///< The reference to the ins interface
    State& state_;                  ///< The reference to the state structure
    Waypoint waypoint_;             ///< The take off waypoint
};

#endif // MISSION_HANDLER_TAKEOFF_HPP_
