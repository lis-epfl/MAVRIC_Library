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
 * \file mission_planner_handler_landing.hpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief The MAVLink mission planner handler for the landing state
 *
 ******************************************************************************/


#ifndef MISSION_PLANNER_HANDLER_LANDING__
#define MISSION_PLANNER_HANDLER_LANDING__

#include "communication/mavlink_message_handler.hpp"
#include "control/mission_planner_handler.hpp"
#include "communication/state.hpp"
#include "control/manual_control.hpp"
#include "control/navigation.hpp"

/*
 * N.B.: Reference Frames and MAV_CMD_NAV are defined in "maveric.h"
 */

class Mission_planner_handler_landing : public Mission_planner_handler
{
public:


    /**
     * \brief   Initialize the landing mission planner handler
     *
     * \param   ins                     The reference to the ins
     * \param   navigation              The reference to the navigation structure
     * \param   state                   The reference to the state structure
     * \param   message_handler         The reference to the mavlink message handler
     */
     Mission_planner_handler_landing(   const INS& ins,
                                        Navigation& navigation,
                                        const ahrs_t& ahrs,
                                        State& state,
                                        Mavlink_message_handler& message_handler);


    /**
     * \brief   The handler for the landing state.
     *
     * \param   mission_planner     The reference to the misison planner that is
     * handling the request.
     */
    virtual void handle(Mission_planner& mission_planner);

    virtual bool init();

    /**
     * \brief   Drives the auto landing procedure from the MAV_CMD_NAV_LAND message long
     *
     * \param   landing_handler         The pointer to the structure of the MAVLink waypoint handler
     * \param   packet                  The pointer to the structure of the MAVLink command message long
     *
     * \return  The MAV_RESULT of the command
     */
    static mav_result_t set_auto_landing(Mission_planner_handler_landing* landing_handler, mavlink_command_long_t* packet);

protected:
    Navigation& navigation_;                                    ///< The reference to the navigation structure
    const ahrs_t& ahrs_;
    State& state_;                                              ///< The reference to the state structure
    Mavlink_message_handler& message_handler_;                  ///< The reference to the mavlink message handler

    /**
     * \brief   Drives the auto-landing navigation behavior
     *
     * \param   mission_planner     The reference to the misison planner that is
     * handling the request.
     */
    void auto_landing_handler(Mission_planner& mission_planner);

private:
    /**
     * \brief   Handles the behavior changes
     */
    void set_behavior();
};







#endif // MISSION_PLANNER_HANDLER_LANDING__
