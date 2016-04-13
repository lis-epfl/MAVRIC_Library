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
 * \file altitude_estimation.cpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief   Altitude estimation
 *
 ******************************************************************************/


#include "sensing/altitude_estimation.hpp"

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Altitude_estimation::Altitude_estimation(Sonar& sonar,
                                         Barometer& barometer,
                                         ahrs_t& ahrs,
                                         altitude_t& altitude,
                                         altitude_estimation_conf_t config):
    Kalman<3,1,1>(std::array<float,3>{{0.0f, 0.0f, 0.0f}},   // x
                  std::array<float,3*3>{{100.0f,   0.0f,   0.0f,
                                           0.0f, 100.0f,   0.0f,
                                           0.0f,   0.0f, 100.0f}},     // P
                  std::array<float,3*3>{{1.0f, 0.004f, 0.000008f,
                                         0.0f, 1.0f,   0.004f,
                                         0.0f, 0.0f,   1.0f}},         // F
                  std::array<float,3*3>{{}}, // Q
                  std::array<float,3>{{1.0f, 0.0f, 0.0f}},               // H
                  std::array<float,1>{{0.0001f}},                         // R
                  std::array<float,3>{{0.000008f, 0.004, 0.0f}}  ),      // B
    sonar_(sonar),
    barometer_(barometer),
    ahrs_(ahrs),
    altitude_(altitude),
    config_(config),
    last_sonar_update_us_(0.0f)
{
    float dt = 0.004f;

    float s_pos = 1.0e-2f; //1.0e-5f;
    float s_vel = 1.0e-1f;
    float s_acc = 1.0e-10f; // 0.0f; //1.0e-20f;

    Mat<3,1> g({s_pos + + s_vel*dt + 0.5f*s_acc*dt*dt,
                s_vel + s_acc*dt,
                s_acc});

    Q_ = g % ~g;
}


bool Altitude_estimation::init(void)
{
    return true;
}

bool Altitude_estimation::update(void)
{
    Kalman<3,1,1>::predict(Mat<1,1>({-ahrs_.linear_acc[2]}));

    if (last_sonar_update_us_ < sonar_.last_update_us() )
    {
        float alt = sonar_.distance();

        if( sonar_.healthy() || alt < 0.3f )
        {
            Kalman<3,1,1>::update(Mat<1,1>({alt}));
            last_sonar_update_us_ = sonar_.last_update_us();
        }
    }

    altitude_.above_sea    = x_(1,0);
    altitude_.above_ground = x_(0,0);

    return true;
}
