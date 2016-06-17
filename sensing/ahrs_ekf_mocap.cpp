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

void Ahrs_mocap::callback(Ahrs_mocap* ahrs_mocap, uint32_t sysid, mavlink_message_t* msg)
{
    ahrs_mocap->R_mocap_(0,0) = ahrs_mocap->config_.R_mocap;
    ahrs_mocap->R_mocap_(1,1) = ahrs_mocap->config_.R_mocap;
    ahrs_mocap->R_mocap_(2,2) = ahrs_mocap->config_.R_mocap;
    ahrs_mocap->R_mocap_(3,3) = ahrs_mocap->config_.R_mocap;

    mavlink_att_pos_mocap_t packet;
    mavlink_msg_att_pos_mocap_decode(msg, &packet);

    // Get timing
    float t = time_keeper_get_us();

    // Create matrices for the update
    Mat<4, 1> z = Mat(0.0f);
    z(0, 0) = packet.q[0];
    z(1, 0) = packet.q[1];
    z(2, 0) = packet.q[2];
    z(3, 0) = packet.q[3];

    Mat<4, 7> H = Mat(0.0f);
    H(0, 3) = 1.0f;
    H(1, 4) = 1.0f;
    H(2, 5) = 1.0f;
    H(3, 6) = 1.0f;

    Mat<4, 4> S = Mat(0.0f);
    Mat<7, 4> K = Mat(0.0f);
    Mat<4, 1> y = Mat(0.0f);

    // Run the ekf update function
    kf::update(ahrs_mocap->x_, ahrs_mocap->P_, z, H, ahrs_mocap->R_mocap_, S, K, y, ahrs_mocap->I_);

    // Update timing
    ahrs_mocap->last_update_us_ = t;
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Ahrs_ekf_mocap::Ahrs_ekf_mocap(const Imu& imu, ahrs_t& ahrs, Mavlink_message_handler& message_handler, const Ahrs_ekf::conf_t config, const Ahrs_ekf_mocap::conf_t config_mocap):
    Ahrs_ekf(imu, ahrs, config),
    config_mocap_(config_mocap)
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
