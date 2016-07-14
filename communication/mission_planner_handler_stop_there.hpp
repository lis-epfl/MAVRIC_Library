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
 * \file mission_planner_handler_stop_there.hpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief The MAVLink mission planner handler for the stop there state
 *
 ******************************************************************************/


#ifndef MISSION_PLANNER_HANDLER_STOP_THERE__
#define MISSION_PLANNER_HANDLER_STOP_THERE__

#include "communication/mission_planner_handler.hpp"
#include "communication/state.hpp"
#include "control/navigation.hpp"
#include "sensing/position_estimation.hpp"

/*
 * N.B.: Reference Frames and MAV_CMD_NAV are defined in "maveric.h"
 */

class Mission_planner_handler_stop_there : public Mission_planner_handler
{
public:


    /**
     * \brief   Initialize the stop there mission planner handler
     *
     * \param   position_estimation     The pointer to the position estimator structure
     * \param   navigation              The reference to the navigation structure
     * \param   state                   The reference to the state structure
     */
     Mission_planner_handler_stop_there(    Position_estimation& position_estimation_,
                                            Navigation& navigation_,
                                            State& state_);


    /**
     * \brief   The handler for the stop on position state.
     *
     * \param   mission_planner     The reference to the misison planner that is
     * handling the request.
     */
    virtual void handle(Mission_planner& mission_planner);

protected:
    Position_estimation& position_estimation_;                   ///< The reference to the position estimation structure
    Navigation& navigation_;                                     ///< The reference to the navigation structure
    State& state_;                                               ///< The reference to the state structure

    /**
     * \brief   Drives the stopping behavior
     *
     * \param   mission_planner     The reference to the mission planner that is
     * handling the request.
     */
    void stopping_handler(Mission_planner& mission_planner);
};







#endif // MISSION_PLANNER_HANDLER_STOP_THERE__