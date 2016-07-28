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
 * \file dubin.h
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *
 * \brief Vector field navigation using Dubin's path
 *
 ******************************************************************************/

#ifndef DUBIN_H_
#define DUBIN_H_

#include <stdint.h>
#include <stdbool.h>

extern "C"
{
#include "util/vectors.h"
#include "util/maths.h"
#include "util/constants.h"
}

typedef enum
{
    DUBIN_INIT = 0,
    DUBIN_CIRCLE1 = 1,
    DUBIN_STRAIGHT = 2,
    DUBIN_CIRCLE2 = 3,
} dubin_state_t;

/*
 * \brief   The Dubin's path structure
 */
typedef struct
{
    float circle_center_1[3];           ///< The center of the first circle
    float tangent_point_1[3];           ///< The tangent point of the first circle
    int8_t sense_1;                     ///< The sense of rotation around the circle
    int8_t radius_1;                    ///< The radius of circle 1
    float circle_center_2[3];           ///< The center of the second circle
    float tangent_point_2[3];           ///< The tangent point of the second circle
    float line_direction[3];            ///< The line created from the two tangent points
    float length;                       ///< The length of the trajectory (to compare multiple Dubin paths)
}dubin_t;

//void dubin_test_code(void);

/**
 * \brief       Vector field to follow a line
 *
 * \param       tvel                The output velocity vector
 * \param       line_dir            The direction of the line
 * \param       line_origin         The origin of the line
 * \param       pos                 The current position of the robot
 * \param       speed               The desired speed
 * \param       one_over_scaling    The inverse of the scaling of the field
 *
 * \return      Return true if everything is ok, false if there is a computational problem.
 */
void dubin_line(float tvel[3], const float line_dir[3], const float line_origin[3], const float pos[3], const float speed, const float one_over_scaling);

/**
 * \brief       Vector field to follow a circle
 *
 * \param       tvel                The output velocity vector
 * \param       circle              The center of the circle
 * \param       radius_mavlink      Radius of the circle to follow (in m), Positive: clockwise
 * \param       speed               The desired speed
 * \param       one_over_scaling    The inverse of the scaling of the field
 * \param       look_ahead_angle    Angular position ahead of the actual position used to compute circular velocity, avoids radius error
 */
void dubin_circle(float tvel[3], const float circle[3], float radius_mavlink, const float pos[3], float speed, float one_over_scaling, float look_ahead_angle=0.0f);

/**
 * \brief       Creating Dubin's path between two waypoints and a sense of rotation
 *
 * \param       wp1                 The starting waypoint
 * \param       wp2                 The ending waypoint
 * \param       d1                  The starting direction
 * \param       d2                  The ending direction
 * \param       sense_mavlink       The rotation sense, Positive: clockwise
 *
 * \return      Return true if everything is ok, false if there is a computational problem.
 */
dubin_t dubin_2d(const float wp1[3], const float wp2[3], const float d1[3], const float d2[3], float sense_mavlink);

#endif /* DUBIN_H_ */
