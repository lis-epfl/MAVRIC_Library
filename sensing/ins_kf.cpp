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
 * \file ins_kf.cpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 * \author Simon Pyroth
 *
 * \brief   Kalman filter for position estimation
 *
 ******************************************************************************/


#include "sensing/ins_kf.hpp"

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

INS_kf::INS_kf(const Gps& gps,
                     const Barometer& barometer,
                     const Sonar& sonar,
                     //const Px4flow_i2c& flow,
                     const ahrs_t& ahrs,
                     const conf_t config):
    Kalman<11,3,3>( {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},                                              // x
                    Mat<11,11>(100, true),                                                          // P
                    { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                                              // F (will be updated)
                      0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                      0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                      0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
                      0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                      0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
                      0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                      0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
                      0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                      0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 },
                    (0.0f),                                                                           // Q (will be updated)
                    { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                                                // H (GPS pos)
                      0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                      0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0 },
                    {SQR(config.sigma_gps_xy), 0,                        0,                           // R (GPS pos)
                     0,                        SQR(config.sigma_gps_xy), 0,
                     0,                        0,                        SQR(config.sigma_gps_z)},
                    Mat<11,3>(0.0f)),                                                                 // B (will be updated)
    INS(config.origin),
    config_(config),
    gps_(gps),
    barometer_(barometer),
    sonar_(sonar),
    //flow_(flow),
    ahrs_(ahrs),
    H_gpsvel_({ 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0}),
    R_gpsvel_({ SQR(config.sigma_gps_velxy),  0,                            0,
                0,                            SQR(config.sigma_gps_velxy),  0,
                0,                            0,                            SQR(config.sigma_gps_velz)}),
    H_baro_({0, 0, -1, 0, 0, 0, 0, 0, 0, 0, -1}),
    R_baro_({ SQR(config.sigma_baro) }),
    H_sonar_({0, 0, -1, 1, 0, 0, 0, 0, 0, 0, 0}),
    R_sonar_({ SQR(config.sigma_sonar) }),
    //H_flow_({0, 0, 0,  0, 1, 0, 0, 0, 0, 0, 0,
    //         0, 0, 0,  0, 0, 1, 0, 0, 0, 0, 0,
    //         0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0}),
    //R_flow_({ 0.0001f, 0,       0,
    //          0,       0.0001f, 0,
     //         0,       0,       0.00001f}),
    last_accel_update_s_(0.0f),
    last_sonar_update_s_(0.0f),
    //last_flow_update_s_(0.0f),
    last_baro_update_s_(0.0f),
    last_gps_pos_update_s_(0.0f),
    last_gps_vel_update_s_(0.0f),
    dt_(0.0f),
    last_update_(0.0f)
{
    P_(7,7) = 0.0f;
    P_(8,8) = 0.0f;
    P_(9,9) = 0.0f;
}


float INS_kf::last_update_s(void) const
{
    float last_update_s = 0.0f;

    last_update_s = maths_f_max(last_update_s, last_sonar_update_s_);
    //last_update_s = maths_f_max(last_update_s, last_flow_update_s_);
    last_update_s = maths_f_max(last_update_s, last_baro_update_s_);
    last_update_s = maths_f_max(last_update_s, last_gps_pos_update_s_);
    last_update_s = maths_f_max(last_update_s, last_gps_vel_update_s_);

    return last_update_s;
}


std::array<float,3> INS_kf::position_lf(void) const
{
    return std::array<float,3>{{x_[0], x_[1], x_[2]}};
}


std::array<float,3> INS_kf::velocity_lf(void) const
{
    return std::array<float,3>{{x_[4], x_[5], x_[6]}};
}


float INS_kf::absolute_altitude(void) const
{
    return -x_[2] + origin_.altitude;
}


bool INS_kf::is_healthy(INS::healthy_t type) const
{
    bool ret = false;

    float now     = time_keeper_get_s();
    float timeout = 1.0f;  // timeout after 1 second

    switch(type)
    {
        case INS::healthy_t::XY_VELOCITY:
            ret = ( ((gps_.fix() >= FIX_2D) && ( (now - last_gps_vel_update_s_) < timeout) )
              //|| ( flow_.healthy()      && ( (now - last_flow_update_s_)    < timeout) ) 
              );

        break;

        case INS::healthy_t::Z_VELOCITY:
            ret = ( ((gps_.fix() >= FIX_3D) && ( (now - last_gps_vel_update_s_) < timeout) ) ||
                    //( flow_.healthy()      && ( (now - last_flow_update_s_)    < timeout) ) ||
                    ( sonar_.healthy()     && ( (now - last_sonar_update_s_)   < timeout) ) );
        break;

        case INS::healthy_t::XYZ_VELOCITY:
            ret = ( ((gps_.fix() >= FIX_3D) && ( (now - last_gps_vel_update_s_) < timeout) )
              //|| ( flow_.healthy()      && ( (now - last_flow_update_s_)    < timeout) )
              );
        break;

        case INS::healthy_t::XY_REL_POSITION:
            ret = ( ((gps_.fix() >= FIX_2D) && ( (now - last_gps_pos_update_s_) < timeout) )
              //|| ( flow_.healthy()      && ( (now - last_flow_update_s_)    < timeout) )
              );
        break;

        case INS::healthy_t::Z_REL_POSITION:
            ret = ( 
                    //( flow_.healthy()      && ( (now - last_flow_update_s_)    < timeout) ) ||
                    ( sonar_.healthy()     && ( (now - last_sonar_update_s_)   < timeout) ) );
        break;

        case INS::healthy_t::XYZ_REL_POSITION:
            ret = ( ((gps_.fix() >= FIX_3D) && ( (now - last_gps_pos_update_s_) < timeout) )
              //|| ( flow_.healthy()      && ( (now - last_flow_update_s_)    < timeout) )
              );
        break;

        case INS::healthy_t::XY_ABS_POSITION:
            ret = ( ((gps_.fix() >= FIX_3D) && ( (now - last_gps_pos_update_s_) < timeout) ) );
        break;

        case INS::healthy_t::Z_ABS_POSITION:
            ret = ( ((gps_.fix() >= FIX_3D) && ( (now - last_gps_pos_update_s_) < timeout) ) ||
                  ( ( (now - last_baro_update_s_) < timeout   ) ) );
        break;

        case INS::healthy_t::XYZ_ABS_POSITION:
            ret = ( ((gps_.fix() >= FIX_3D) && ( (now - last_gps_pos_update_s_) < timeout) ));
        break;
    }

    return ret;
}


bool INS_kf::update(void)
{
    // Prediction step
    if (ahrs_.internal_state == AHRS_READY)
    {
        if (last_accel_update_s_ < ahrs_.last_update_s)
        {
            // Update the delta time (in second)
            float now       = time_keeper_get_s();
            dt_             = now - last_update_;
            last_update_    = now;

            // Make the prediciton
            predict_kf();

            // update timimg
            last_accel_update_s_ = ahrs_.last_update_s;
        }
    }
    else
    {
        // Reset covariance matrix
        P_ = Mat<11,11>(100.0f, true);
        P_(7,7) = 0.0f;
        P_(8,8) = 0.0f;
        P_(9,9) = 0.0f;

        // Update last time to avoid glitches at initilaization
        last_update_ = time_keeper_get_s();
    }


    // Update step
    // Measure from gps
    if (gps_.healthy())
    {
        // GPS Position
        if (last_gps_pos_update_s_ < gps_.last_position_update_us()/1e6)
        {
            // Get local position from gps
            //local_position_t gps_local;
            gps_global = gps_.position_gf();
            ori = origin();
            coord_conventions_global_to_local_position(gps_global, origin(), gps_local);

            // Update the measurement noise if needed
            if(!config_.constant_covar)
            {
                // TODO: Implement this!

                // Recompute the measurement noise matrix
                R_ = Mat<3,3>({ SQR(config_.sigma_gps_xy), 0,                         0,
                                0,                         SQR(config_.sigma_gps_xy), 0,
                                0,                         0,                         SQR(config_.sigma_gps_z)});
            }

            // Run kalman update using default matrices
            Kalman<11,3,3>::update({gps_local[0], gps_local[1], gps_local[2]});

            // Update timing
            last_gps_pos_update_s_ = gps_.last_position_update_us()/1e6;
        }

        // GPS velocity
        if (last_gps_vel_update_s_ < gps_.last_velocity_update_us()/1e6)
        {
            // Update the measurement noise if needed
            if(!config_.constant_covar)
            {
                // TODO: Implement this!

                // Recompute the measurement noise matrix
                R_gpsvel_ = Mat<3,3>({ SQR(config_.sigma_gps_velxy),  0,                            0,
                                       0,                             SQR(config_.sigma_gps_velxy), 0,
                                       0,                             0,                            SQR(config_.sigma_gps_velz)});
            }

            // Run kalman update
            gps_velocity = gps_.velocity_lf();
            Kalman<11,3,3>::update(Mat<3,1>(gps_.velocity_lf()),
                                   H_gpsvel_,
                                   R_gpsvel_);

            // Update timing
            last_gps_vel_update_s_ = gps_.last_velocity_update_us()/1e6;
        }
    }

    // Measure from barometer
    // TODO: Add healthy function into barometer
    if(true)
    {
       if (last_baro_update_s_ < barometer_.last_update_us()/1e6)
       {
          // Update the measurement noise if needed
          if(!config_.constant_covar)
          {
              // TODO: Implement this!

              // Recompute the measurement noise matrix
              R_baro_ = Mat<1,1>({ SQR(config_.sigma_baro) });
          }

          // Run kalman Update
          z_baro = barometer_.altitude_gf() - origin_.altitude;
          Kalman<11,3,3>::update(Mat<1,1>(z_baro),
                                 H_baro_,
                                 R_baro_);
       
          // Update timing
          last_baro_update_s_ = barometer_.last_update_us()/1e6;
       }
    }

    // Measure from sonar
    if (sonar_.healthy())
    {
       if (last_sonar_update_s_ < sonar_.last_update_us()/1e6)
       {
          // Update the measurement noise if needed
          if(!config_.constant_covar)
          {
              // TODO: Implement this!

              // Recompute the measurement noise matrix
              R_sonar_ = Mat<1,1>({ SQR(config_.sigma_sonar) });
          }

          // Run kalman Update
          z_sonar = sonar_.distance();
          Kalman<11,3,3>::update(Mat<1,1>(z_sonar),
                                 H_sonar_,
                                 R_sonar_);

          // Update timing
          last_sonar_update_s_ = sonar_.last_update_us()/1e6f;
       }
    }

    // Measure from optic-flow
    //if (flow.healthy())
    // {
    //     if (last_flow_update_s_ < flow_.last_update_s())
    //     {
    //         // run kalman update on velocity
    //         float vel_lf[3];
    //         float vel_bf[3] = {-flow_.velocity_y(), flow_.velocity_x(), 0.0f};
    //         quaternions_rotate_vector(ahrs_.qe, vel_bf, vel_lf);
    //         Kalman<11,3,3>::update(Mat<3,1>({vel_lf[0], vel_lf[1], flow_.ground_distance()}),
    //         // Kalman11,3,3>::update(Mat<3,1>({vel_bf[0], vel_bf[1], flow_.ground_distance()}),
    //                                H_flow_,
    //                                R_flow_);

    //         // Update timing
    //         last_flow_update_s_ = flow_.last_update_s();
    //     }
    // }

    return true;
}


void INS_kf::predict_kf(void)
{
    // Recompute the variable model matrices
    // Get time
    //float dt = 0.004; //dt_;
    float dt = dt_;
    float dt2 = (dt*dt)/2.0f;

    // Get attitude quaternion
    quat_t q = ahrs_.qe;
    float q0 = q.s;
    float q1 = q.v[0];
    float q2 = q.v[1];
    float q3 = q.v[2];

    // Compute coefficients
    float ax = q0*q0 + q1*q1 - q2*q2 - q3*q3;
    float bx = 2.0f*(-q0*q3 + q1*q2);
    float cx = 2.0f*(q0*q2 + q1*q3);
    float ay = 2.0f*(q0*q3 + q1*q2);
    float by = q0*q0 - q1*q1 + q2*q2 - q3*q3;
    float cy = 2.0f*(-q0*q1 + q2*q3);
    float az = 2.0f*(-q0*q2 + q1*q3);
    float bz = 2.0f*(q0*q1 + q2*q3);
    float cz = q0*q0 - q1*q1 - q2*q2 - q3*q3;

    // Model dynamics (modify only the non-constant terms)
    // TODO: Test block insertion method
    // F_.insert(Mat<3,3>({ dt,  0,  0,
    //                      0,   dt, 0,
    //                      0,   0,  dt }),
    //           0,
    //           4);
    // F_.insert(Mat<3,3>({ -ax*dt2, -bx*dt2,  -cx*dt2,
    //                      -ay*dt2, -by*dt2,  -cy*dt2,
    //                      -az*dt2, -bz*dt2,  -cz*dt2 }),
    //           0,
    //           7);
    // F_.insert(Mat<3,3>({ -ax*dt,  -bx*dt, -cx*dt,
    //                      -ay*dt,  -by*dt, -cy*dt,
    //                      -az*dt,  -bz*dt, -cz*dt}),
    //           4,
    //           7);
    F_(0,4) = dt;
    F_(1,5) = dt;
    F_(2,6) = dt;
    F_(0,7) = -ax*dt2;
    F_(0,8) = -bx*dt2;
    F_(0,9) = -cx*dt2;
    F_(1,7) = -ay*dt2;
    F_(1,8) = -by*dt2;
    F_(1,9) = -cy*dt2;
    F_(2,7) = -az*dt2;
    F_(2,8) = -bz*dt2;
    F_(2,9) = -cz*dt2;
    F_(4,7) = -ax*dt;
    F_(4,8) = -bx*dt;
    F_(4,9) = -cx*dt;
    F_(5,7) = -ay*dt;
    F_(5,8) = -by*dt;
    F_(5,9) = -cy*dt;
    F_(6,7) = -az*dt;
    F_(6,8) = -bz*dt;
    F_(6,9) = -cz*dt;

    // Input (modify only the non-constant terms)
    // B_.insert(Mat<3,3>({ ax*dt2, bx*dt2, cx*dt2,
    //                      ay*dt2, by*dt2, cy*dt2,
    //                      az*dt2, bz*dt2, cz*dt2 }),
    //           0,
    //           0);
    // B_.insert(Mat<3,3>({ ax*dt, bx*dt,  cx*dt,
    //                      ay*dt, by*dt,  cy*dt,
    //                      az*dt, bz*dt,  cz*dt }),
    //           4,
    //           0);
    B_(0,0) = ax*dt2;
    B_(0,1) = bx*dt2;
    B_(0,2) = cx*dt2;
    B_(1,0) = ay*dt2;
    B_(1,1) = by*dt2;
    B_(1,2) = cy*dt2;
    B_(2,0) = az*dt2;
    B_(2,1) = bz*dt2;
    B_(2,2) = cz*dt2;
    B_(4,0) = ax*dt;
    B_(4,1) = bx*dt;
    B_(4,2) = cx*dt;
    B_(5,0) = ay*dt;
    B_(5,1) = by*dt;
    B_(5,2) = cy*dt;
    B_(6,0) = az*dt;
    B_(6,1) = bz*dt;
    B_(6,2) = cz*dt;


    // Recompute process noise matrix
    // Coefficients
    float axx = ax*ax + bx*bx + cx*cx;
    float ayy = ay*ay + by*by + cy*cy;
    float azz = az*az + bz+bz + cz*cz;
    float axy = ax*ay + bx*by + cx*cy;
    float axz = ax*az + bx*bz + cx*cz;
    float ayz = ay*az + by*bz + cy*cz;
    float sz2 = SQR(config_.sigma_z_gnd);
    float sa2 = SQR(config_.sigma_bias_acc);
    float sb2 = SQR(config_.sigma_bias_baro);
    float su2 = SQR(config_.sigma_acc);
    float sau2 = sa2 + su2;

    // Time constants
    float dt520 = (dt*dt*dt*dt*dt)/20.0f;
    float dt48 = (dt*dt*dt*dt)/8.0f;
    float dt36 = (dt*dt*dt)/6.0f;
    float dt33 = (dt*dt*dt)/3.0f;
    float dt22 = (dt*dt)/2.0f;

    // Matrix
    // TODO: Improve this, using fixed matrix and multiply only terms with axx, axy, ...
    Q_ = Mat<11,11>({ dt520*axx*sau2, dt520*axy*sau2, dt520*axz*sau2, 0,      dt48*axx*sau2,  dt48*axy*sau2,  dt48*axz*sau2,  -dt36*ax*sa2, -dt36*bx*sa2, -dt36*cx*sa2, 0,
                      dt520*axy*sau2, dt520*ayy*sau2, dt520*ayz*sau2, 0,      dt48*axy*sau2,  dt48*ayy*sau2,  dt48*ayz*sau2,  -dt36*ay*sa2, -dt36*by*sa2, -dt36*cy*sa2, 0,
                      dt520*axz*sau2, dt520*ayz*sau2, dt520*azz*sau2, 0,      dt48*axz*sau2,  dt48*ayz*sau2,  dt48*azz*sau2,  -dt36*az*sa2, -dt36*bz*sa2, -dt36*cz*sa2, 0,
                      0,              0,              0,              dt*sz2, 0,              0,              0,              0,            0,            0,            0,
                      dt48*axx*sau2,  dt48*axy*sau2,  dt48*axz*sau2,  0,      dt33*axx*sau2,  dt33*axy*sau2,  dt33*axz*sau2,  -dt22*ax*sa2, -dt22*bx*sa2, -dt22*cx*sa2, 0,
                      dt48*axy*sau2,  dt48*ayy*sau2,  dt48*ayz*sau2,  0,      dt33*axy*sau2,  dt33*ayy*sau2,  dt33*ayz*sau2,  -dt22*ay*sa2, -dt22*by*sa2, -dt22*cy*sa2, 0,
                      dt48*axz*sau2,  dt48*ayz*sau2,  dt48*azz*sau2,  0,      dt33*axz*sau2,  dt33*ayz*sau2,  dt33*azz*sau2,  -dt22*az*sa2, -dt22*bz*sa2, -dt22*cz*sa2, 0,
                      -dt36*ax*sa2,   -dt36*ay*sa2,   -dt36*az*sa2,   0,      -dt22*ax*sa2,   -dt22*ay*sa2,   -dt22*az*sa2,   dt*sa2,       0,            0,            0,
                      -dt36*bx*sa2,   -dt36*by*sa2,   -dt36*bz*sa2,   0,      -dt22*bx*sa2,   -dt22*by*sa2,   -dt22*bz*sa2,   0,            dt*sa2,       0,            0,
                      -dt36*cx*sa2,   -dt36*cy*sa2,   -dt36*cz*sa2,   0,      -dt22*cx*sa2,   -dt22*cy*sa2,   -dt22*cz*sa2,   0,            0,            dt*sa2,       0,
                      0,              0,              0,              0,      0,              0,              0,              0,            0,            0,            dt*sb2 });





    // Compute default KF prediciton step (using local accelerations as input)
    predict({ahrs_.linear_acc[0], ahrs_.linear_acc[1], ahrs_.linear_acc[2]});
}
