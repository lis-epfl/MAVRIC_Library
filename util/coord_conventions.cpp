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
#include "util/constants.hpp"
#include "util/print_util.hpp"
#include "util/quick_trig.hpp"

#include <cmath>

extern "C"
{
#include "util/maths.h"
}

// Definitions constants as double for precise conversion
#define EARTH_RADIUS_DOUBLE     6378137.0
#define PI_DOUBLE               3.141592653589793
#define MATH_DEG_TO_RAD_DOUBLE  (PI_DOUBLE/180.0)
#define MATH_RAD_TO_DEG_DOUBLE  (180.0/PI_DOUBLE)


void coord_conventions_local_to_global_position(const local_position_t& input, const global_position_t& origin, global_position_t& output)
{
    output.latitude     = origin.latitude  + MATH_RAD_TO_DEG_DOUBLE * ((double)(input[0]) / EARTH_RADIUS_DOUBLE);
    output.longitude    = origin.longitude + MATH_RAD_TO_DEG_DOUBLE * ((double)(input[1]) / (EARTH_RADIUS_DOUBLE * cos(MATH_DEG_TO_RAD_DOUBLE * output.latitude)));
    output.altitude     = -input[2] + origin.altitude;
}


void coord_conventions_global_to_local_position(const global_position_t& position, const global_position_t& origin, local_position_t& output)
{
    double small_radius;

    small_radius    = cos(MATH_DEG_TO_RAD_DOUBLE * position.latitude) * EARTH_RADIUS_DOUBLE;
    output[X]       = (float)(sin(MATH_DEG_TO_RAD_DOUBLE * (position.latitude  - origin.latitude))  * EARTH_RADIUS_DOUBLE);
    output[Y]       = (float)(sin(MATH_DEG_TO_RAD_DOUBLE * (position.longitude - origin.longitude)) * small_radius);
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


void coord_conventions_rpy_from_quaternion(const quat_t& qe, float rpy[3])
{
    rpy[0] = atan2(2 * (qe.s * qe.v[0] + qe.v[1] * qe.v[2]) , (qe.s * qe.s - qe.v[0] * qe.v[0] - qe.v[1] * qe.v[1] + qe.v[2] * qe.v[2]));
    rpy[1] = -asin(2 * (qe.v[0] * qe.v[2] - qe.s * qe.v[1]));
    rpy[2] = atan2(2 * (qe.s * qe.v[2] + qe.v[0] * qe.v[1]) , (qe.s * qe.s + qe.v[0] * qe.v[0] - qe.v[1] * qe.v[1] - qe.v[2] * qe.v[2]));
}


float coord_conventions_get_yaw(quat_t qe)
{
    return  atan2(2 * (qe.s * qe.v[2] + qe.v[0] * qe.v[1]) , (qe.s * qe.s + qe.v[0] * qe.v[0] - qe.v[1] * qe.v[1] - qe.v[2] * qe.v[2]));
}
