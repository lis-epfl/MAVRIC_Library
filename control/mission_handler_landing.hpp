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
 * \file mission_handler_landing.hpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief The MAVLink mission planner handler for the landing state
 *
 ******************************************************************************/


#ifndef MISSION_HANDLER_LANDING__
#define MISSION_HANDLER_LANDING__

#include "communication/state.hpp"
#include "control/mission_handler.hpp"
#include "control/navigation.hpp"

/*
 * N.B.: Reference Frames and MAV_CMD_NAV are defined in "maveric.h"
 */

class Mission_handler_landing : public Mission_handler
{
public:


    /**
     * \brief   Initialize the landing mission planner handler
     *
     * \param   ins                     The reference to the ins
     * \param   navigation              The reference to the navigation structure
     * \param   state                   The reference to the state structure
     */
     Mission_handler_landing(   const INS& ins,
                                Navigation& navigation,
                                State& state);

    /**
     * \brief   Checks if the waypoint is a landing waypoint
     *  
     * \details     Checks if the inputted waypoint is a:
     *                  MAV_CMD_NAV_LAND
     *
     * \param   wpt                 The waypoint class
     *
     * \return  Can handle
     */
    bool can_handle(const Waypoint& wpt);

    /**
     * \brief   Sets up this handler class for a first time initialization
     *  
     * \details     Records the waypoint reference and sets the mav mode
     *
     * \param   mission_planner     The mission planner class
     * \param   wpt                 The waypoint class
     *
     * \return  Success
     */
    bool setup(Mission_planner& mission_planner, const Waypoint& wpt);

    /**
     * \brief   Handles the mission every iteration
     *  
     * \details     
     *
     * \param   mission_planner     The mission planner class
     */
    void handle(Mission_planner& mission_planner);

    /**
     * \brief   Return false
     *  
     * \details     This returns false as we do not want the drone to take off
     *              immediately after landing
     *
     * \param   mission_planner     The mission planner class
     *
     * \return  False
     */
    bool is_finished(Mission_planner& mission_planner);

    /**
     * \brief   Limits the vertical velocity
     *
     * \details Limits the vertical velocity during the descent to ground state
     *
     * \param   control     Control command reference to change
     */
    void modify_control_command(control_command_t& control);

protected:
    const Waypoint* waypoint_;                                        ///< The waypoint that we are landing under
    Waypoint landing_waypoint_;                                 ///< The waypoint that we want our drone to go
    bool is_landed_;                                            ///< Boolean flag stating that we have finished the landing procedure

    const INS& ins_;                                            ///< The reference to the ins interface
    Navigation& navigation_;                                    ///< The reference to the navigation structure
    State& state_;                                              ///< The reference to the state structure
};







#endif // MISSION_HANDLER_LANDING__
