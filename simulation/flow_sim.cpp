/*******************************************************************************
 * Copyright (c) 2009-2015, MAV'RIC Development Team
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
 * \file flow_sim.cpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief   Simulated Optic Flow sensors
 *
 ******************************************************************************/

#include "simulation/flow_sim.hpp"

extern "C"
{
#include "util/quick_trig.h"
}

Flow_sim::Flow_sim(Dynamic_model& dynamic_model, raytracing::World& world, float orientation_azimuth):
    dynamic_model_(dynamic_model),
    world_(world)
{
    last_update_us = 0;
    of_count = ray_count_;
    for (uint32_t i = 0; i < of_count; i++)
    {
        // Init flow
        of.x[i] = 0.0f;
        of.y[i] = 0.0f;

        // Init flow location
        float azimuth = orientation_azimuth + (1.0f + 2.0f * i) * (PI / 180.0f); // looks right by default
        of_loc.x[i] = azimuth;
        of_loc.y[i] = 0.0f;

        // Init rays
        rays_[i].set_direction(Vector3f{quick_trig_cos(azimuth),
                                        quick_trig_sin(azimuth),
                                        0.0f});

        // Init OF jacobian for each ray
        jacob_[i] = {quick_trig_cos(of_loc.x[i]) * quick_trig_sin(of_loc.y[i]),  quick_trig_sin(of_loc.x[i]) * quick_trig_sin(of_loc.y[i]), -quick_trig_cos(of_loc.y[i]),
                     quick_trig_sin(of_loc.x[i]) / quick_trig_cos(of_loc.y[i]), -quick_trig_cos(of_loc.x[i]) / quick_trig_cos(of_loc.y[i]),  0.0f};
    }
}

bool Flow_sim::update(void)
{
    raytracing::Ray           ray_tmp;
    raytracing::Intersection  inter_tmp;
    raytracing::Object*       obj_tmp = NULL;
    Mat<2,1>                  of_tmp;

    // Get current velocity
    quat_t att = dynamic_model_.attitude();
    float vel_lf[3] = {dynamic_model_.velocity_lf()[0], dynamic_model_.velocity_lf()[1], dynamic_model_.velocity_lf()[2]};
    float vel_bf[3];
    quaternions_rotate_vector(att, vel_lf, vel_bf);
    Mat<3,1> vel;
    vel[0] = vel_bf[0];
    vel[1] = vel_bf[1];
    vel[2] = vel_bf[2];


    for (uint32_t i = 0; i < of_count; i++)
    {
        // Translate ray
        local_position_t pos_lf = dynamic_model_.position_lf();
        ray_tmp.set_origin(Vector3f{pos_lf.pos[0], pos_lf.pos[1], pos_lf.pos[2]});

        // Rotate ray
        float orient_bf[3] = {rays_[i].direction()[0], rays_[i].direction()[1], rays_[i].direction()[2]};
        float orient_lf[3];
        quaternions_rotate_vector( quaternions_inverse(att), orient_bf, orient_lf);
        ray_tmp.set_direction(Vector3f{orient_lf[0], orient_lf[1], orient_lf[2]});

        // Test intersection
        float proximity;
        if (world_.intersect(ray_tmp, inter_tmp, obj_tmp))
        {
            proximity = 1.0f / inter_tmp.distance();
        }
        else
        {
            proximity = 0.0f;
        }

        // Compute optic flow
        // of_tmp = jacob_[i] % (vel * proximity);
        // of.x[i] = of_tmp[0];
        // of.y[i] = of_tmp[1];

        of.x[i] = (vel[0] * proximity) * quick_trig_sin(of_loc.x[i]);
        of.y[i] = 0.0f;
    }

    return true;
}
