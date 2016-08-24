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
 * \file mission_handler_critical_landing.hxx
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief   The MAVLink mission planner handler functions for the critical 
 *          landing state
 *
 ******************************************************************************/


#ifndef MISSION_HANDLER_CRITICAL_LANDING_HXX__
#define MISSION_HANDLER_CRITICAL_LANDING_HXX__

template <class T1, class T2>
Mission_handler_critical_landing<T1, T2>::Mission_handler_critical_landing<T1, T2>( T1& desc_to_small_alt_controller,
                                                                                    T2& desc_to_ground_controller,
                                                                                    const INS& ins,
                                                                                    Navigation& navigation,
                                                                                    State& state):
            Mission_handler_landing(desc_to_small_alt_controller, desc_to_ground_controller, ins, navigation, state)
{
}

template <class T1, class T2>
bool Mission_handler_critical_landing<T1, T2>::can_handle(const Waypoint& wpt) const
{
    bool handleable = false;

    uint16_t cmd = wpt.command();
    if (cmd == MAV_CMD_NAV_CRITICAL_LAND)
    {
        handleable = true;
    }

    return handleable;
}

#endif // MISSION_HANDLER_CRITICAL_LANDING_HXX__
