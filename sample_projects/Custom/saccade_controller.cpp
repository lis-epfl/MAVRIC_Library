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
 * \file saccade_controller.c
 *
 * \author Darius Merk
 *
 * \brief   Saccade controller for indoors collision free navigation based on optic flow
 *
 ******************************************************************************/


#include "saccade_controller.hpp"

extern "C"
{
    #include "util/maths.h"
    #include "util/quick_trig.h"
}





//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------
Saccade_controller::Saccade_controller( flow_t& flow_left,
                                        flow_t& flow_right,
                                        attitude_command_t&  attitude_command,
                                        saccade_controller_conf_t config ):
  flow_left_(flow_left),
  flow_right_(flow_right),
  attitude_command_(attitude_command)
{
    // Init members
    gain_            = config.gain_;
    threshold_       = config.threshold_;
    goal_direction_  = config.goal_direction_;

    // 125 points along the 160 pixels of the camera, start at pixel number 17 finish at number 142 such that the total angle covered by the 125 points is 140.625 deg.
    float angle_between_points = (140.625 / N_points);

    // Init azimuth angles
    for (uint32_t i = 0; i < N_points; ++i)
    {
        azimuth_[i]             = (-160.875 + i * angle_between_points) * (PI / 180.0f);
        azimuth_[i + N_points]  = (  19.125 + i * angle_between_points) * (PI / 180.0f);
    }
}

bool Saccade_controller::init(void)
{
    return true;
}

/*void Saccade_controller::set_direction(heading_angle)
{
  Voir comment définir ca
}*/



// #include <stdio.h>

bool Saccade_controller::update()
{
    // Intermediate variables :
    // can is the norm of the comanv vector,
    // nearest object direction gives the angle in radians to the nearest object,
    // cad gives the collision avoidance direction, opposite to the nearest
    // object direction.
    float nearest_object_direction  = 0.0f;
    float movement_direction        = 0.0f;

    // Sigmoid function for direction choice, it takes the can, a threshold and
    // a gain and describes how important it is for the drone to perform a saccade
    float weighted_function = 1.0f;

    // Quaternion given to attitude controller for the saccade
    quat_t quat_yaw_command;

    // Random number generation for the noise, the value of the noise is between 0 and 0.5. A new number is generated at each time.
    // ATTENTION CHECK THAT THE NOISE IS RANDOM AND ISN'T 10 TIMES THE SAME IN 1S FOR EXAMPLE
    // float noise = 0.0f;

    // Calculate for both left and right the sum of the relative nearnesses
    // which are each given by RN = OF/sin(angle),
    for (uint32_t i = 0; i < N_points; ++i)
    {
        // relative_nearness_[i]             = flow_left_.of.x[i]  / quick_trig_sin(azimuth_[i]);
        // relative_nearness_[i + N_points]  = flow_right_.of.x[i] / quick_trig_sin(azimuth_[i + N_points]);

        // test
        relative_nearness_[i]             = flow_left_.of.x[i];
        // relative_nearness_[i]             = quick_trig_sin(azimuth_[i]);

        // printf("%f\n", quick_trig_sin(azimuth_[i]));
        // if (quick_trig_sin(azimuth_[i]) != 0.0f)
        // {

        relative_nearness_[i]             = 1.0f / (10.0f + quick_trig_sin(azimuth_[i]) );
        // relative_nearness_[i]             = 1.0f / azimuth_[i];
        // relative_nearness_[i]             = 1.0f / relative_nearness_[i];

        // }


        // TODO
        // Precompute 1.0f / quick_sin(azimuth[i]) to optimize this loop
    }
    // printf("\n");



    // // Calculate the comanv's x and y components, to then calculate can and NOD.
    // float comanv_x = 0.0f;
    // float comanv_y = 0.0f;
    // for (uint32_t i = 0; i < 2*N_points; i++)
    // {
    //     comanv_x += quick_trig_cos(azimuth_[i]) * relative_nearness_[i];
    //     comanv_y += quick_trig_sin(azimuth_[i]) * relative_nearness_[i];
    //
    //     // TODO
    //     // Same as above, precompute cos and sin
    // }




   //Calculation of the can and cad

    // can_ = sqrt(pow(comanv_x , 2) + pow(comanv_y , 2));
    //
    // nearest_object_direction = atan(comanv_y/comanv_x);
    //
    // weighted_function = 1 / (1 + pow(can_/threshold_ , - gain_));
    //
    // cad_ = nearest_object_direction + PI;
    //

    // tmp
    can_ = flow_left_.of.x[50];
    cad_ = flow_right_.of.x[50];


    //Calculation of the movement direction (in radians)
    //
    // movement_direction = weighted_function * cad_ + (1-weighted_function) * (goal_direction_ + noise);

    //Transformation of the movement direction into a quaternion

    aero_attitude_t attitude;
    attitude.rpy[0] = 0.0f;
    attitude.rpy[1] = 0.0f;
    attitude.rpy[2] = movement_direction;

    quat_yaw_command = coord_conventions_quaternion_from_aero(attitude);

    //Attitude command given by the required movement direction

    attitude_command_.quat = quat_yaw_command;

    return true;
}
