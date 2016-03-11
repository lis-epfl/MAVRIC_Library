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
 * \file sonar_sim.cpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief   Simulation for sonars
 *
 ******************************************************************************/


#include "simulation/sonar_sim.hpp"

extern "C"
{
#include "util/constants.h"
}

Sonar_sim::Sonar_sim(Dynamic_model& dynamic_model, sonar_sim_conf_t config):
    dynamic_model_(dynamic_model),
    config_(config),
    distance_(0.0f),
    velocity_(0.0f),
    healthy_(true)
{}


bool Sonar_sim::init(void)
{
    return true;
}


bool Sonar_sim::update(void)
{
    /**
     * H: scalar (altitude)
     * D: scalar (measure)
     * z: down vector   ||z||=1
     * u: sensor vector ||u||=1
     *
     * With
     * Altitude = H.z
     * Distance = D.u
     *
     * We have
     * D = H / (u.z)
     */

    bool success = true;

    // Update dynamic model
    success &= dynamic_model_.update();

    // Get current attitude and position
    quat_t           attitude = dynamic_model_.attitude();
    local_position_t position = dynamic_model_.position_lf();

    // Get orientation in global frame
    std::array<float, 3> orientation = orientation_bf();
    float orient_bf[3] = {orientation[X], orientation[Y], orientation[Z]};
    float orient_lf[3];
    quaternions_rotate_vector(attitude, orient_bf, orient_lf);

    // Get u.z  (u: sensor orientation, z: vertical down)
    // We keep only values with ratio > 0.707, ie. when the angle between
    // vertical and sensor is lower than 45 degrees
    float down[3] = {0.0f, 0.0f, 1.0f};
    float ratio = vectors_scalar_product(orient_lf, down);

    if (ratio > 0.707)
    {
        float new_distance = - position.pos[Z] / ratio;

        if (new_distance > config_.min_distance && new_distance < config_.max_distance)
        {
            float dt_s = (last_update_us_ - dynamic_model_.last_update_us()) / 1000000.0f;

            // Update velocity
            if (healthy_ && dt_s != 0.0f)
            {
                float new_velocity = (new_distance - distance_) / dt_s;

                //discard sonar new_velocity estimation if it seems too big
                if (maths_f_abs(new_velocity) > 20.0f)
                {
                    new_velocity = 0.0f;
                }

                velocity_ =   0.5f * velocity_ + 0.5f * new_velocity;
            }
            else
            {
                velocity_ = 0.0f;
            }

            last_update_us_ = dynamic_model_.last_update_us();
            healthy_        = true;
        }
        else
        {
            velocity_   = 0.0f;
            healthy_    = false;
        }

        // Update current distance even if not healthy
        distance_ = new_distance;
    }
    else
    {
        distance_   = config_.max_distance;
        velocity_   = 0.0f;
        healthy_    = false;
    }


    return success;
}


const float& Sonar_sim::last_update_us(void) const
{
    return last_update_us_;
}


const std::array<float, 3>& Sonar_sim::orientation_bf(void) const
{
    return config_.orientation_bf;
}


const float& Sonar_sim::distance(void) const
{
    return distance_;
}


const float& Sonar_sim::velocity(void) const
{
    return velocity_;
}


const bool& Sonar_sim::healthy(void) const
{
    return healthy_;
}
