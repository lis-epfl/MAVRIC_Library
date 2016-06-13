/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
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
 * \file ahrs_mocap.cpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief Records the ahrs quaternion from the motion capture system and updates
 * the ahrs vector accordingly.
 *
 ******************************************************************************/

#include "sensing/ahrs_mocap.hpp"
#include "hal/common/time_keeper.hpp"

extern "C"
{

}


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void Ahrs_mocap::callback(Ahrs_mocap* ahrs_mocap, uint32_t sysid, mavlink_message_t* msg)
{
    mavlink_att_pos_mocap_t packet;
    mavlink_msg_att_pos_mocap_decode(msg, &packet);

    // Get timing
    float t = time_keeper_get_us();

    // Update ahrs quaternion
    ahrs_mocap->ahrs_.qe.s = packet.q[0];
    ahrs_mocap->ahrs_.qe.v[0] = packet.q[1];
    ahrs_mocap->ahrs_.qe.v[1] = packet.q[2];
    ahrs_mocap->ahrs_.qe.v[2] = packet.q[3];

    // Update timing
    ahrs_mocap->last_update_us_ = t;
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Ahrs_mocap::Ahrs_mocap(Mavlink_message_handler& message_handler, ahrs_t& ahrs):
    ahrs_(ahrs)
{
    // Add callbacks for waypoint handler messages requests
    Mavlink_message_handler::msg_callback_t callback;

    callback.message_id     = MAVLINK_MSG_ID_ATT_POS_MOCAP; // 69
    callback.sysid_filter   = MAVLINK_BASE_STATION_ID;
    callback.compid_filter  = MAV_COMP_ID_ALL;
    callback.function       = (Mavlink_message_handler::msg_callback_func_t) &Ahrs_mocap::callback;
    callback.module_struct  = (Mavlink_message_handler::handling_module_struct_t)        this;

    message_handler.add_msg_callback(&callback);
}
