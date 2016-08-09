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


#ifndef MISSION_PLANNER_HANDLER_HOLD_POSITION__
#define MISSION_PLANNER_HANDLER_HOLD_POSITION__

#include "control/mission_handler.hpp"
#include "communication/state.hpp"
#include "control/navigation.hpp"

/*
 * N.B.: Reference Frames and MAV_CMD_NAV are defined in "maveric.h"
 */

class Mission_handler_hold_position : public Mission_handler
{
public:


    /**
     * \brief   Initialize the hold position mission planner handler
     *
     * \param   ins                     The reference to the ins
     * \param   navigation              The reference to the navigation structure
     * \param   state                   The reference to the state structure
     */
     Mission_handler_hold_position( const INS& ins,
                                    Navigation& navigation,
                                    State& state);

    /**
     * \brief   Checks if the waypoint is a hold position waypoint
     *  
     * \details     Checks if this is a:
                        MAV_CMD_NAV_LOITER_UNLIM
                        MAV_CMD_NAV_LOITER_TIME
                        MAV_CMD_NAV_LOITER_TO_ALT
     *
     * \param   mission_planner     The mission planner class
     * \param   wpt                 The waypoint class
     *
     * \return  Can handle
     */
    bool can_handle(Mission_planner& mission_planner, Waypoint& wpt);

    /**
     * \brief   Sets up this handler class for a first time initialization
     *  
     * \details     Stores the waypoint reference and records the starting
                    time
     *
     * \param   mission_planner     The mission planner class
     * \param   wpt                 The waypoint class
     *
     * \return  Success
     */
    bool setup(Mission_planner& mission_planner, Waypoint& wpt);

    /**
     * \brief   Handles the mission every iteration
     *  
     * \details     Sets the waypoint goal to the setup waypoint
     *
     * \param   mission_planner     The mission planner class
     */
    void handle(Mission_planner& mission_planner);

    /**
     * \brief   Checks if the handler has finished the request of the waypoint
     *  
     * \details     Returns true or false based on the specifications of
                    the inputted waypoint
     *
     * \param   mission_planner     The mission planner class
     *
     * \return  Is finished
     */
    bool is_finished(Mission_planner& mission_planner);

protected:
    Waypoint& waypoint_;                ///< Reference to the inputted waypoint
    uint64_t start_time_;               ///< The start time of the waypoint hold

    const INS& ins;                     ///< The reference to the ins structure
    Navigation& navigation_;            ///< The reference to the navigation structure
    State& state_;                      ///< The reference to the state structure
};







#endif // MISSION_PLANNER_HANDLER_HOLD_POSITION__
