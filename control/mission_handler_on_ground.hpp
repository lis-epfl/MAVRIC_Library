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

#include "control/mission_handler.hpp"
#include "control/navigation.hpp"

/*
 * N.B.: Reference Frames and MAV_CMD_NAV are defined in "maveric.h"
 */

class Mission_handler_on_ground : public Mission_handler
{
public:


    /**
     * \brief   Initialize the on ground mission planner handler
     *
     * \param   navigation              The reference to the navigation class
     */
     Mission_handler_on_ground(Navigation& navigation);


    /**
     * \brief   Checks if the waypoint is on the ground
     *  
     * \details     DOES NOT CURRENTLY CHECK IF WE ARE ON GROUND
     *
     * \param   wpt                 The waypoint class
     *
     * \return  Can handle
     */
    bool can_handle(Waypoint& wpt);

    /**
     * \brief   Does nothing
     *  
     * \details     Does nothing
     *
     * \param   mission_planner     The mission planner class
     * \param   wpt                 The waypoint class
     *
     * \return  True
     */
    bool setup(Mission_planner& mission_planner, Waypoint& wpt);

    /**
     * \brief   Does nothing
     *  
     * \details     Does nothing
     *
     * \param   mission_planner     The mission planner class
     */
    void handle(Mission_planner& mission_planner);

    /**
     * \brief   Returns false
     *  
     * \details     Returns false as the on ground state should never
                    end without user input from somewhere elses
     *
     * \param   mission_planner     The mission planner class
     *
     * \return  False
     */
    bool is_finished(Mission_planner& mission_planner);

    /**
     * \brief   Sets the control command to 0
     *
     * \param   control     Control command
     */
    void modify_control_command(control_command_t& control);

protected:
    Navigation& navigation_;                                    ///< The reference to the navigation structure
};







#endif // MISSION_HANDLER_ON_GROUND__
