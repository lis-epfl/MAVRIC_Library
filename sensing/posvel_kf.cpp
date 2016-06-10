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
 * \file posvel_kf.cpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief   Kalman filter for position estimation
 *
 ******************************************************************************/


#include "sensing/posvel_kf.hpp"

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Posvel_kf::Posvel_kf(const Gps& gps,
                     const Barometer& barometer,
                     const Sonar& sonar,
                     const Px4flow_i2c& flow,
                     const ahrs_t& ahrs,
                     posvel_t& posvel,
                     const conf_t config):
    Kalman<8,3,3>({0, 0, 0, 0, 0, 0, 0, 0},                                                         // x
                  Mat<8,8>(100, true),                                                              // P
                  { 1, 0, 0, 0, config.dt, 0,         0,         0,
                    0, 1, 0, 0, 0,         config.dt, 0,         0,
                    0, 0, 1, 0, 0,         0,         config.dt, 0,
                    0, 0, 0, 1, 0,         0,         0,         0,
                    0, 0, 0, 0, 1,         0,         0,         0,
                    0, 0, 0, 0, 0,         1,         0,         0,
                    0, 0, 0, 0, 0,         0,         1,         0,
                    0, 0, 0, 0, 0,         0,         0,         1},                                // F
                  { 0, 0, 0, 0, 0,     0,     0,     0,
                    0, 0, 0, 0, 0,     0,     0,     0,
                    0, 0, 0, 0, 0,     0,     0,     0,
                    0, 0, 0, 0, 0,     0,     0,     0,
                    0, 0, 0, 0, 0.05f, 0,     0,     0,
                    0, 0, 0, 0, 0,     0.05f, 0,     0,
                    0, 0, 0, 0, 0,     0,     0.05f, 0,
                    0, 0, 0, 0, 0,     0,     0,     0.01f},                                                    // Q
                  { 1, 0, 0, 0, 0, 0, 0, 0,
                    0, 1, 0, 0, 0, 0, 0, 0,
                    0, 0, 1, 0, 0, 0, 0, 0 },                                                       // H1
                  {SQR(config.sigma_gps_xy), 0,                        0,
                   0,                        SQR(config.sigma_gps_xy), 0,                           // R1
                   0,                        0,                        SQR(config.sigma_gps_z)},
                  {0.5f*SQR(config.dt), 0,                   0,
                   0,                   0.5f*SQR(config.dt), 0,
                   0,                   0,                   0.5f*SQR(config.dt),
                   0,                   0,                   0,
                   config.dt,           0,                   0,
                   0,                   config.dt,           0,
                   0,                   0,                   config.dt,
                   0,                   0,                   0}),                                // B
    gps_(gps),
    barometer_(barometer),
    sonar_(sonar),
    flow_(flow),
    ahrs_(ahrs),
    posvel_(posvel),
    config_(config),
    H_gpsvel_({ 0, 0, 0, 0, 1, 0, 0, 0,
                0, 0, 0, 0, 0, 1, 0, 0,
                0, 0, 0, 0, 0, 0, 1, 0, }),
    R_gpsvel_({ 5, 0,    0,
                0,    5, 0,
                0,    0,    5}),
    H_baro_({0, 0, 1, 0, 0, 0, 0, 1}),
    R_baro_({100.0f}),
    H_sonar_({0, 0, 1, -1, 0, 0, 0, 0}),
    R_sonar_({0.01f}),
    H_flow_({0, 0, 0,  0, 1, 0, 0, 0,
             0, 0, 0,  0, 0, 1, 0, 0,
             0, 0, 1, -1, 0, 0, 0, 0}),
    R_flow_({ 0.02f, 0,     0,
              0,     0.02f, 0,
              0,     0,     0.001f}),
    last_accel_update_s_(0.0f),
    last_sonar_update_s_(0.0f),
    last_flow_update_s_(0.0f),
    last_baro_update_s_(0.0f),
    last_gps_pos_update_s_(0.0f),
    last_gps_vel_update_s_(0.0f)
{}


bool Posvel_kf::init(void)
{
    return true;
}


bool Posvel_kf::update(void)
{
    // Prediction
    if (ahrs_.internal_state == AHRS_READY)
    {
        if (last_accel_update_s_ < ahrs_.last_update_s)
        {
            // run kalman prediciton using accelerometers
            predict({ahrs_.linear_acc[0], ahrs_.linear_acc[1], ahrs_.linear_acc[2]});
            // predict({0.0f, 0.0f, 0.0f});

            // update timimg
            last_accel_update_s_ = ahrs_.last_update_s;
        }
    }
    else
    {
        // Reset covariance matrix
        P_ = Mat<8,8>(100.0f, true);
    }

    // Correction from gps
    if (gps_.healthy())
    {
        // GPS Position
        if (last_gps_pos_update_s_ < gps_.last_position_update_us()*1e6)
        {
            // Get local position from gps
            local_position_t local_pos = coord_conventions_global_to_local_position(gps_.position_gf(),
                                                                                    config_.home);

            // run kalman update using default matrices
            Kalman<8,3,3>::update({local_pos.pos[0], local_pos.pos[1], local_pos.pos[2]});

            // Update timing
            last_gps_pos_update_s_ = gps_.last_position_update_us()*1e6;
        }

        // GPS velocity
        if (last_gps_vel_update_s_ < gps_.last_velocity_update_us()*1e6)
        {
            // run kalman update
            Kalman<8,3,3>::update(Mat<3,1>(gps_.velocity_lf()),
                                  H_gpsvel_,
                                  R_gpsvel_);

            // Update timing
            last_gps_vel_update_s_ = gps_.last_velocity_update_us()*1e6;
        }
    }

    // Correction from barometer
    // if (barometer_.healthy())
    // {
      //  if (last_baro_update_s_ < barometer_.last_update_us()*1e6)
      //  {
      //     // run kalman Update
      //     Kalman<8,3,3>::update(Mat<1,1>(config_.home.altitude - barometer_.altitude_gf()),
      //                           H_baro_,
      //                           R_baro_);
       //
      //     // Update timing
      //     last_baro_update_s_ = barometer_.last_update_us()*1e6;
      //  }
    // }

    // Correction from sonar
    if (sonar_.healthy())
    {
       if (last_sonar_update_s_ < sonar_.last_update_us()*1e6)
       {
          // run kalman Update
          Kalman<8,3,3>::update(Mat<1,1>(sonar_.distance()),
                                H_sonar_,
                                R_sonar_);

          // Update timing
          last_sonar_update_s_ = sonar_.last_update_us()*1e6;
       }
    }

    //if (flow.healthy())
    {
        if (last_flow_update_s_ < flow_.last_update_s())
        {
            // run kalman update on velocity
            float vel_bf[3] = {-flow_.velocity_y(), flow_.velocity_x(), 0.0f};
            // float vel_lf[3];
            // quaternions_rotate_vector(quaternions_inverse(ahrs_.qe), vel_bf, vel_lf);
            // Kalman<8,3,3>::update(Mat<3,1>({vel_lf[0], vel_lf[1], flow_.ground_distance()}),
            Kalman<8,3,3>::update(Mat<3,1>({vel_bf[0], vel_bf[1], flow_.ground_distance()}),
                                  H_flow_,
                                  R_flow_);

            // Update timing
            last_flow_update_s_ = flow_.last_update_s();
        }

    }

    // Write output
    posvel_.pos[0] = x_[0];
    posvel_.pos[1] = x_[1];
    posvel_.pos[2] = x_[2] - x_[3];
    posvel_.vel[0] = x_[4];
    posvel_.vel[1] = x_[5];
    posvel_.vel[2] = x_[6];

    return true;
}
