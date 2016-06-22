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
 * \file coord_conventions.c
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief Coordinate conventions
 *
 ******************************************************************************/


#include "util/coord_conventions.hpp"
#include <math.h>
#include "util/maths.h"
#include "util/print_util.h"
#include "util/quick_trig.h"
#include "util/constants.hpp"

void coord_conventions_local_to_global_position(const local_position_t& input, const global_position_t& origin, global_position_t& output)
{
    output.latitude     = origin.latitude  + maths_rad_to_deg(input[0] / EARTH_RADIUS);
    output.longitude    = origin.longitude + maths_rad_to_deg(input[1] / (EARTH_RADIUS * cos(maths_deg_to_rad(output.latitude))));
    output.altitude     = -input[2] + origin.altitude;
}


void coord_conventions_global_to_local_position(const global_position_t& position, const global_position_t& origin, local_position_t& output)
{
    double small_radius;

    small_radius    = cos(maths_deg_to_rad(position.latitude)) * EARTH_RADIUS;
    output[X]       = (float)(sin(maths_deg_to_rad((position.latitude - origin.latitude))) * EARTH_RADIUS);
    output[Y]       = (float)(sin(maths_deg_to_rad((position.longitude - origin.longitude))) * small_radius);
    output[Z]       = (float)(-(position.altitude - origin.altitude));
}


aero_attitude_t coord_conventions_quat_to_aero(quat_t qe)
{
    aero_attitude_t aero;

    aero.rpy[0] = atan2(2 * (qe.s * qe.v[0] + qe.v[1] * qe.v[2]) , (qe.s * qe.s - qe.v[0] * qe.v[0] - qe.v[1] * qe.v[1] + qe.v[2] * qe.v[2]));
    aero.rpy[1] = -asin(2 * (qe.v[0] * qe.v[2] - qe.s * qe.v[1]));
    aero.rpy[2] = atan2(2 * (qe.s * qe.v[2] + qe.v[0] * qe.v[1]) , (qe.s * qe.s + qe.v[0] * qe.v[0] - qe.v[1] * qe.v[1] - qe.v[2] * qe.v[2]));

    return aero;
}


quat_t coord_conventions_quaternion_from_aero(aero_attitude_t aero)
{
    quat_t quat;

    // intermediate values
    float cr, cp, cy, sr, sp, sy;
    cr = quick_trig_cos(aero.rpy[0] / 2.0f);
    cp = quick_trig_cos(aero.rpy[1] / 2.0f);
    cy = quick_trig_cos(aero.rpy[2] / 2.0f);
    sr = quick_trig_sin(aero.rpy[0] / 2.0f);
    sp = quick_trig_sin(aero.rpy[1] / 2.0f);
    sy = quick_trig_sin(aero.rpy[2] / 2.0f);


    quat.s      = (cr * cp * cy) + (sr * sp * sy);
    quat.v[0]   = (sr * cp * cy) - (cr * sp * sy);
    quat.v[1]   = (cr * sp * cy) + (sr * cp * sy);
    quat.v[2]   = (cr * cp * sy) - (sr * sp * cy);

    return quat;
}


quat_t coord_conventions_quaternion_from_rpy(const float rpy[3])
{
    quat_t quat;

    // intermediate values
    float cr, cp, cy, sr, sp, sy;
    cr = quick_trig_cos(rpy[0] / 2.0f);
    cp = quick_trig_cos(rpy[1] / 2.0f);
    cy = quick_trig_cos(rpy[2] / 2.0f);
    sr = quick_trig_sin(rpy[0] / 2.0f);
    sp = quick_trig_sin(rpy[1] / 2.0f);
    sy = quick_trig_sin(rpy[2] / 2.0f);


    quat.s      = (cr * cp * cy) + (sr * sp * sy);
    quat.v[0]   = (sr * cp * cy) - (cr * sp * sy);
    quat.v[1]   = (cr * sp * cy) + (sr * cp * sy);
    quat.v[2]   = (cr * cp * sy) - (sr * sp * cy);

    return quat;
}

float coord_conventions_get_yaw(quat_t qe)
{
    return  atan2(2 * (qe.s * qe.v[2] + qe.v[0] * qe.v[1]) , (qe.s * qe.s + qe.v[0] * qe.v[0] - qe.v[1] * qe.v[1] - qe.v[2] * qe.v[2]));
}
