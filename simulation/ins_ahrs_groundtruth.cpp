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
 * \file ins_ahrs_groundtruth.hpp
 *
 * \author MAV'RIC Team
 * \author Basil Huber

 * \brief   Mockup Inertial Navigation System (INS) and Attitude Heading Reference System (AHRS) providing groundtruth
 *          for position and velocity, attitude, acceleration and rates, obtained from the dynamic model
 *
 ******************************************************************************/


#include "ins_ahrs_groundtruth.hpp"

INS_AHRS_groundtruth::INS_AHRS_groundtruth(Dynamic_model& model, const conf_t& config):
    INS(config.origin),
    model_(model),
    attitude_(quat_t{1.0f, {0.0f, 0.0f, 0.0f}}),
    angular_speed_{{0.0f, 0.0f, 0.0f}},
    linear_acc_{{0.0f, 0.0f, 0.0f}},
    last_update_s_(0.0f)
{
    update();
}


bool INS_AHRS_groundtruth::update()
{
    model_.update();

    /* udpate internal INS values */
    last_update_s_ = model_.last_update_us()/1e6;
    position_lf_ = model_.position_lf();
    velocity_lf_ = model_.velocity_lf();

    /* calculate absolute altitude */
    global_position_t global_pos;
    coord_conventions_local_to_global_position(model_.position_lf(), origin(), global_pos);
    absolute_altitude_ = global_pos.altitude;

    /* update ahrs */
    attitude_       = model_.attitude();
    angular_speed_  = model_.angular_velocity_bf();
    linear_acc_     = model_.acceleration_bf();

    return true;
}

float INS_AHRS_groundtruth::last_update_s(void) const
{
    return last_update_s_;
}


quat_t INS_AHRS_groundtruth::attitude(void) const
{
    return attitude_;
}


std::array<float,3> INS_AHRS_groundtruth::angular_speed(void) const
{
    return angular_speed_;
}


std::array<float,3> INS_AHRS_groundtruth::linear_acceleration(void) const
{
    return linear_acc_;
}
