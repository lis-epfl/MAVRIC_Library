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
 * \file mission_planner_handler_navigating.hpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief The MAVLink mission planner handler for the navigating state
 *
 ******************************************************************************/


#ifndef MISSION_PLANNER_HANDLER_NAVIGATING__
#define MISSION_PLANNER_HANDLER_NAVIGATING__

#include "communication/mission_planner_handler.hpp"
#include "communication/state.hpp"
#include "communication/mavlink_message_handler.hpp"
#include "control/manual_control.hpp"
#include "control/navigation.hpp"
#include "sensing/position_estimation.hpp"

/*
 * N.B.: Reference Frames and MAV_CMD_NAV are defined in "maveric.h"
 */

class Mission_planner_handler_navigating : public Mission_planner_handler
{
public:


    /**
     * \brief   Initialize the navigating mission planner handler
     *
     * \param   position_estimation     The pointer to the position estimator structure
     * \param   navigation              The reference to the navigation structure
     * \param   state                   The reference to the state structure
     * \param   mavlink_stream          The reference to the MAVLink stream structure
     */
     Mission_planner_handler_navigating(    Position_estimation& position_estimation,
                                            Navigation& navigation_,
                                            State& state_,
                                            const Mavlink_stream& mavlink_stream)


    /**
     * \brief   The handler for the takeoff state.
     */
    virtual void handle();

protected:
    Position_estimation& position_estimation_;                   ///< The reference to the position estimation object
    Navigation& navigation_;                                     ///< The reference to the navigation object
    State& state_;                                               ///< The reference to the state object
    const Mavlink_stream& mavlink_stream_;                      ///< The reference to the mavlink object

    /**
     * \brief   Drives the GPS navigation procedure
     *
     */
    void waypoint_navigating_handler(bool reset_hold_wpt);
};







#endif // MISSION_PLANNER_HANDLER_NAVIGATING__
