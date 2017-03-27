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
* \file geofence_cylinder.cpp
*
* \author MAV'RIC Team
* \author Julien Lecoeur
*
* \brief  Geofence with cylindrical shape
*
******************************************************************************/


#include "status/geofence_cylinder.hpp"
#include "sensing/ins.hpp"

extern "C"
{
#include "util/maths.h"
#include "util/vectors.h"
}

Geofence_cylinder::Geofence_cylinder(conf_t config):
    config_(config)
{}


bool Geofence_cylinder::is_allowed(const global_position_t& position) const
{
    if (config_.enabled == false)
    {
        // Geofence is disabled -> all locations are allowed
        return true;
    }
    else
    {
        // Get position in local frame
        local_position_t position_lf;
        local_position_t center_lf;
        coord_conventions_global_to_local_position(position,        INS::origin(), position_lf);
        coord_conventions_global_to_local_position(config_.center,  INS::origin(), center_lf);

        // Get distance to fence center
        float dist_xy_sqr = SQR(position_lf[X] - center_lf[X]) + SQR(position_lf[Y] - center_lf[Y]);

        // Test if inside
        bool is_inside = (dist_xy_sqr < SQR(config_.radius)) && (position_lf[Z] > (center_lf[Z] - config_.height));

        if (config_.allowed_inside == true)
        {
            return is_inside;
        }
        else
        {
            return !is_inside;
        }
    }
}

bool Geofence_cylinder::closest_border(const global_position_t& current_position, global_position_t& border_position, float& distance) const
{
    if (config_.enabled == false)
    {
        // Geofence is diabled -> closest safe point is the current position
        border_position = current_position;
        distance        = 0.0f;
    }
    else
    {

        // Get position in local frame
        local_position_t position_lf;
        local_position_t center_lf;
        coord_conventions_global_to_local_position(current_position, INS::origin(), position_lf);
        coord_conventions_global_to_local_position(config_.center,   INS::origin(), center_lf);

        // Unit vector from cylinder center to current position
        float u[3];
        for (size_t i = 0; i < 3; i++)
        {
            u[i] = position_lf[i] - center_lf[i];
        }
        vectors_normalize(u, u);

        // Get closest barrier point on cylinder
        local_position_t border_cyl;
        border_cyl[X] = center_lf[X] + config_.radius * u[X];
        border_cyl[X] = center_lf[Y] + config_.radius * u[Y];
        border_cyl[Z] = maths_f_max(center_lf[Z] - config_.height, position_lf[Z]);
        float border_cyl_dist_sqr = SQR(border_cyl[X] - position_lf[X])
                                  + SQR(border_cyl[Y] - position_lf[Y])
                                  + SQR(border_cyl[Z] - position_lf[Z]);

        // Get closest barrier point on cylinder top plane
        local_position_t border_top;
        border_top[X] = position_lf[X];
        border_top[Y] = position_lf[Y];
        border_top[Z] = center_lf[Z] - config_.height;
        float border_top_dist_sqr = SQR(border_top[X] - position_lf[X])
                                  + SQR(border_top[Y] - position_lf[Y])
                                  + SQR(border_top[Z] - position_lf[Z]);

        if (border_cyl_dist_sqr < border_top_dist_sqr)
        {
            // Convert to global frame
            coord_conventions_local_to_global_position(border_cyl, INS::origin(), border_position);

            // Return shortest distance
            distance = maths_fast_sqrt(border_cyl_dist_sqr);
        }
        else
        {
            // Convert to global frame
            coord_conventions_local_to_global_position(border_top, INS::origin(), border_position);

            // Return shortest distance
            distance = maths_fast_sqrt(border_top_dist_sqr);
        }
    }

    return true;
}
