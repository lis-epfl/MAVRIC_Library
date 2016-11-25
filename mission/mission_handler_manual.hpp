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
 * \file mission_handler_manual.hpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 * \author Julien Lecoeur
 *
 * \brief The mission handler for the manual control state
 *
 ******************************************************************************/


#ifndef MISSION_HANDLER_MANUAL_HPP_
#define MISSION_HANDLER_MANUAL_HPP_

#include "mission/mission_handler.hpp"


/**
 * \brief The mission handler for the manual control state
 */
class Mission_handler_manual : public Mission_handler
{
public:
    /**
     * \brief   Initialize the manual mission planner handler
     */
     Mission_handler_manual();


    /**
     * \brief   Checks if the waypoint is on the ground
     *
     * \details     Checks if the inputted waypoint is a:
     *                  MAV_CMD_NAV_MANUAL_CTRL
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
     * \brief   Handles the mission every iteration
     *
     * \details     Does nothing and returns MISSION_FINISHED for the status code.
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
     * \brief   Returns that the mission state is in MANUAL_CTRL
     *
     * \return  Mission handler's mission state
     */
    virtual Mission_planner::internal_state_t handler_mission_state() const;

protected:

};

#endif // MISSION_HANDLER_MANUAL_HPP_
