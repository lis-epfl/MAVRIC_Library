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
 * \file mission_handler_critical_landing.hpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief The MAVLink mission planner handler for the critical landing state
 *
 ******************************************************************************/


#ifndef MISSION_HANDLER_CRITICAL_LANDING__
#define MISSION_HANDLER_CRITICAL_LANDING__

#include "communication/state.hpp"
#include "mission/mission_handler_landing.hpp"
#include "mission/navigation.hpp"

/*
 * N.B.: Reference Frames and MAV_CMD_NAV are defined in "maveric.h"
 */

class Mission_handler_critical_landing : public Mission_handler_landing
{
public:


    /**
     * \brief   Initialize the landing mission planner handler
     *
     * \param   ins                     The reference to the ins
     * \param   navigation              The reference to the navigation structure
     * \param   state                   The reference to the state structure
     */
     Mission_handler_critical_landing(  const INS& ins,
                                        Navigation& navigation,
                                        State& state);

    /**
     * \brief   Checks if the waypoint is a landing waypoint
     *  
     * \details     Checks if the inputted waypoint is a:
     *                  MAV_CMD_NAV_CRITICAL_LAND
     *
     * \param   wpt                 The waypoint class
     *
     * \return  Can handle
     */
    virtual bool can_handle(const Waypoint& wpt) const;
};







#endif // MISSION_HANDLER_CRITICAL_LANDING__
