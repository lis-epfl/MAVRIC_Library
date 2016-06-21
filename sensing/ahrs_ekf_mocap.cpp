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
 * \file ahrs_ekf_mocap.cpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief Extended Kalman Filter attitude estimation, mixing accelerometer and magnetometer
 * x[0] : bias_x
 * x[1] : bias_y
 * x[2] : bias_z
 * x[3] : q0
 * x[4] : q1
 * x[5] : q2
 * x[6] : q3
 *
 * Takes into account the motion capture ahrs quaternion
 *
 ******************************************************************************/

#include "sensing/ahrs_ekf_mocap.hpp"
#include "hal/common/time_keeper.hpp"
#include "util/kalman.hpp"

extern "C"
{
#include "util/constants.h"
#include "util/print_util.h"
#include "util/maths.h"
#include "util/vectors.h"
#include "util/quaternions.h"
#include "util/coord_conventions.h"
}


using namespace mat;

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------
void Ahrs_ekf_mocap::callback(Ahrs_ekf_mocap* ahrs_ekf_mocap, uint32_t sysid, mavlink_message_t* msg)
{
    ahrs_ekf_mocap->R_mocap_ = Mat<4, 4>(ahrs_ekf_mocap->config_.R_mocap, true);

    mavlink_att_pos_mocap_t packet;
    mavlink_msg_att_pos_mocap_decode(msg, &packet);

    // Get timing
    float t = time_keeper_get_us();

    // Create matrices for the update
    ahrs_ekf_mocap->z_(0, 0) = packet.q[0];
    ahrs_ekf_mocap->z_(1, 0) = packet.q[1];
    ahrs_ekf_mocap->z_(2, 0) = packet.q[2];
    ahrs_ekf_mocap->z_(3, 0) = packet.q[3];

    // Run the ekf update function
    ahrs_ekf_mocap->ahrs_ekf_.Kalman<7,0,3>::update(ahrs_ekf_mocap->z_, ahrs_ekf_mocap->H_, ahrs_ekf_mocap->R_mocap_);

    // Update timing
    ahrs_ekf_mocap->last_update_us_ = t;
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Ahrs_ekf_mocap::Ahrs_ekf_mocap(Mavlink_message_handler& message_handler, Ahrs_ekf& ahrs_ekf, const conf_t config_):
    ahrs_ekf_(ahrs_ekf),
    message_handler_(message_handler),
    is_init_(false),
    config_(config_)
{
    R_mocap_ = Mat<4, 4>(config_.R_mocap, true);

    H_ = Mat<4, 7>(0.0f);
    H_(0, 3) = 1.0f;
    H_(1, 4) = 1.0f;
    H_(2, 5) = 1.0f;
    H_(3, 6) = 1.0f;

    z_ = Mat<4, 1>(0.0f);
}

bool Ahrs_ekf_mocap::init()
{
    if (!is_init_)
    {
        // Add callbacks for waypoint handler messages requests
        Mavlink_message_handler::msg_callback_t callback;

        callback.message_id     = MAVLINK_MSG_ID_ATT_POS_MOCAP; // 69
        callback.sysid_filter   = MAVLINK_BASE_STATION_ID;
        callback.compid_filter  = MAV_COMP_ID_ALL;
        callback.function       = (Mavlink_message_handler::msg_callback_func_t) &Ahrs_ekf_mocap::callback;
        callback.module_struct  = (Mavlink_message_handler::handling_module_struct_t)        this;

        is_init_ = message_handler_.add_msg_callback(&callback);
    }

    return is_init_;
}
