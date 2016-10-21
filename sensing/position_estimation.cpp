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
 * \file position_estimation.cpp
 *
 * \author MAV'RIC Team
 *
 * \brief This file performs the 3D position estimation, either by direct
 * integration or with correction with the GPS and pos_est->barometer
 *
 ******************************************************************************/


#include "sensing/position_estimation.hpp"
#include "hal/common/time_keeper.hpp"
#include "util/print_util.hpp"

extern "C"
{
#include "util/maths.h"
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Position_estimation::Position_estimation(State& state, const Barometer& barometer, const Sonar& sonar, const Gps& gps, const Px4flow_i2c& flow, const ahrs_t& ahrs, const conf_t config) :
    local_position_(std::array<float,3>{{0.0f, 0.0f, 0.0f}}),
    vel_{0.0f,0.0f,0.0f},
    config_(config),
    ahrs_(ahrs),
    state_(state),
    gps_(gps),
    barometer_(barometer),
    sonar_(sonar),
    flow_(flow),
    dt_s_(0.0f),
    last_update_s_(0.0f),
    last_gps_pos_update_us_(0),
    last_gps_vel_update_us_(0),
    last_barometer_update_us_(0),
    last_sonar_update_us_(0),
    last_flow_update_us_(0),
    is_gps_pos_initialized_(false),
    fence_position_(std::array<float,3>{{0.0f, 0.0f, 0.0f}}),
    barometer_bias_(0.0f),
    is_barometer_calibrated_(false)
{
    // default GPS origin
    set_origin(config.origin);
}


bool Position_estimation::update(void)
{
    // Updat timing
    float now      = time_keeper_get_s();
    dt_s_          = now - last_update_s_;
    last_update_s_ = now;


    if (ahrs_.internal_state == AHRS_READY)
    {
        check_first_gps_fix();

        if (state_.reset_position)
        {
            state_.reset_position = false;
            reset_velocity_altitude();
        }

        integration();

        if (config_.use_gps)
        {
            correction_from_gps_pos();
            correction_from_gps_vel();
        }

        if (config_.use_baro)
        {
            correction_from_barometer();
        }

        if (config_.use_sonar)
        {
            correction_from_sonar();
        }

        if (config_.use_flow)
        {
            correction_from_flow();
        }

        if (state_.is_armed())
        {
            fence_control();
        }

        return true;
    }
    else
    {
        return false;
    }
}


void Position_estimation::reset_velocity_altitude()
{
    // Correct barometer bias
    if (is_gps_pos_initialized_)
    {
        calibrate_barometer();

        print_util_dbg_print("Offset of the barometer set to the GPS altitude, new altitude of:");
        print_util_dbg_print_num(barometer_.altitude_gf() - barometer_bias_, 10);
        print_util_dbg_print(" ( ");
        print_util_dbg_print_num(local_position_[2], 10);
        print_util_dbg_print("  ");
        print_util_dbg_print_num(origin().altitude, 10);
        print_util_dbg_print(" )\r\n");
    }

    // reset position estimator
    for (int32_t i = 0; i < 3; i++)
    {
        vel_[i] = 0.0f;
    }
}


Position_estimation::fence_violation_state_t Position_estimation::get_fence_violation_state() const
{
    fence_violation_state_t fence_violation_state = IN_FENCE;
    if(state_.out_of_fence_2)
    {
        fence_violation_state = OUTSIDE_FENCE2;
    }
    else if(state_.out_of_fence_1)
    {
        fence_violation_state = OUTSIDE_FENCE1;
    }
    return fence_violation_state;
}


float Position_estimation::last_update_s(void) const
{
    return last_update_s_;
}


std::array<float,3> Position_estimation::position_lf(void) const
{
    return local_position_;
}


std::array<float,3> Position_estimation::velocity_lf(void) const
{
    return std::array<float, 3>{{vel_[0], vel_[1], vel_[2]}};
}


float Position_estimation::absolute_altitude(void) const
{
    return (origin().altitude - local_position_[Z]);
}


bool Position_estimation::is_healthy(INS::healthy_t __attribute__((unused)) type) const
{
    return true;
}


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void Position_estimation::integration()
{
    // Get velocity in body frame
    float vel_bf[3];
    quaternions_rotate_vector(quaternions_inverse(ahrs_.qe), vel_, vel_bf);

    // Integrate velocity using accelerometer
    for (uint32_t i = 0; i < 3; i++)
    {
        vel_bf[i] += ahrs_.linear_acc[i] * dt_s_;
    }

    // Get new velocity in local frame
    quaternions_rotate_vector(ahrs_.qe, vel_bf, vel_);

    // Integrate position using estimated velocity
    for (uint32_t i = 0; i < 3; i++)
    {
        local_position_[i] += vel_[i] * dt_s_;
    }
}


void Position_estimation::correction_from_gps_pos(void)
{
    float gps_gain     = 0.0f;
    float pos_error[3] = { 0.0f, 0.0f, 0.0f };

    if (gps_.healthy() == true)
    {
        // Enable gps correction
        gps_gain = 1.0f;

        if (last_gps_pos_update_us_ < gps_.last_position_update_us() + config_.timeout_gps_us)
        {
            last_gps_pos_update_us_ = gps_.last_position_update_us();

            // get local position from GPS
            local_position_t gps_pos;
            coord_conventions_global_to_local_position(gps_.position_gf(), origin(), gps_pos);

            // Compute position error and velocity error from gps
            for (uint32_t i = 0; i < 3; i++)
            {
                pos_error[i] = gps_pos[i] - local_position_[i];
            }
        }
    }
    else
    {
        // Disable gps correction
        gps_gain = 0.0f;
    }

    // Apply error correction to position estimates
    for (uint32_t i = 0; i < 3; i++)
    {
        local_position_[i] += config_.kp_gps_pos[i] * gps_gain * pos_error[i] * dt_s_;
    }
}


void Position_estimation::correction_from_gps_vel(void)
{
    float gain         = 0.0f;
    float vel_error[3] = { 0.0f, 0.0f, 0.0f };

    if (gps_.healthy() == true)
    {
        // Enable gps correction
        gain = 1.0f;

        if (last_gps_vel_update_us_ < gps_.last_velocity_update_us() + config_.timeout_gps_us)
        {
            last_gps_vel_update_us_ = gps_.last_velocity_update_us();

            // Compute position error and velocity error from gps
            for (uint32_t i = 0; i < 3; i++)
            {
                vel_error[i] = gps_.velocity_lf()[i] - vel_[i];
            }
        }
    }
    else
    {
        // Disable gps correction
        gain = 0.0f;
    }

    // Apply error correction to velocity estimates
    for (uint32_t i = 0; i < 3; i++)
    {
        vel_[i] += config_.kp_gps_vel[i] * gain * vel_error[i] * dt_s_;
    }
}


void Position_estimation::correction_from_barometer(void)
{
    float baro_alt_error = 0.0f;
    float baro_vel_error = 0.0f;
    float gain_alt;
    float gain_vel;

    if (is_barometer_calibrated_)
    {
        // Enable correction from barometer
        gain_alt = config_.kp_baro_alt;
        gain_vel = config_.kp_baro_vel;

        // altimeter correction
        if (last_barometer_update_us_ < barometer_.last_update_us() + config_.timeout_baro_us)
        {
            baro_alt_error = - (barometer_.altitude_gf() - origin().altitude - barometer_bias_) - local_position_[Z];
            baro_vel_error = barometer_.vertical_speed_lf() - vel_[Z];

            last_barometer_update_us_ = barometer_.last_update_us();
        }
    }
    else
    {
        // Disable correction from barometer
        gain_alt = 0.0f;
        gain_vel = 0.0f;

        // Wait for gps to initialized as we need an absolute altitude
        if (is_gps_pos_initialized_)
        {
            // Correct barometer bias
            calibrate_barometer();
        }
    }

    // Apply corrections
    local_position_[Z] += gain_alt * baro_alt_error * dt_s_;
    vel_[Z]            += gain_vel * baro_vel_error * dt_s_;
}


void Position_estimation::correction_from_sonar(void)
{
    float sonar_alt_error = 0.0f;
    float sonar_vel_error = 0.0f;
    float gain_alt        = 0.0f;
    float gain_vel        = 0.0f;

    if (sonar_.healthy())
    {
        if (last_sonar_update_us_ < sonar_.last_update_us() + config_.timeout_sonar_us)
        {
            gain_alt        = 1.0f;
            gain_vel        = 1.0f;
            sonar_alt_error = - sonar_.distance() - local_position_[Z];
            sonar_vel_error = - sonar_.velocity() - vel_[Z];
        }
    }

    // Apply corrections
    local_position_[Z] += config_.kp_sonar_alt * gain_alt * sonar_alt_error * dt_s_;
    vel_[Z]            += config_.kp_sonar_vel * gain_vel * sonar_vel_error * dt_s_;
}


void Position_estimation::correction_from_flow(void)
{
    float vel_error[3] = {0.0f, 0.0f, 0.0f};
    float alt_error    = 0.0f;
    float gain         = 0.0f;

    if (flow_.healthy())
    {
        if (last_flow_update_us_ < (flow_.last_update_s() * 1e6f) + config_.timeout_flow_us)
        {
            last_flow_update_us_ = (flow_.last_update_s() * 1e6f);
            gain           = 1.0f;

            // Get velocity in NED frame
            float vel_lf[3];
            float vel_bf[3] = {flow_.velocity_x(), flow_.velocity_y(), flow_.velocity_z()};
            quaternions_rotate_vector(coord_conventions_quaternion_from_rpy(0.0f,
                                                                            0.0f,
                                                                            coord_conventions_get_yaw(ahrs_.qe)),
                                      vel_bf,
                                      vel_lf);

            // Compute errors
            vel_error[0] = vel_lf[X] - vel_[X];
            vel_error[1] = vel_lf[Y] - vel_[Y];
            vel_error[2] = vel_lf[Z] - vel_[Z];
            alt_error    = - flow_.ground_distance() - local_position_[Z];
        }
    }

    // Apply corrections
    vel_[X]            += config_.kp_flow_vel  * gain * vel_error[X] * dt_s_;
    vel_[Y]            += config_.kp_flow_vel  * gain * vel_error[Y] * dt_s_;
    vel_[Z]            += config_.kp_sonar_vel * gain * vel_error[Z] * dt_s_;
    local_position_[Z] += config_.kp_sonar_alt * gain * alt_error    * dt_s_;
}


void Position_estimation::check_first_gps_fix()
{
    if ((is_gps_pos_initialized_ == false) && (gps_.healthy() == true))
    {
        is_gps_pos_initialized_ = true;

        last_gps_pos_update_us_ = gps_.last_position_update_us();
        last_gps_vel_update_us_ = gps_.last_velocity_update_us();

        set_origin(gps_.position_gf());

        for (int32_t i = 0; i < 3; i++)
        {
            local_position_[i] = 0.0f;
            vel_[i] = 0.0f;
        }

        print_util_dbg_print("GPS position initialized!\r\n");
    }
}

void Position_estimation::fence_control()
{
    float dist_xy_sqr, dist_z_sqr;
    dist_xy_sqr = SQR(local_position_[X] - fence_position_[X]) + SQR(local_position_[Y] - fence_position_[Y]);
    dist_z_sqr = SQR(local_position_[Z] - fence_position_[Z]);

    if (dist_xy_sqr > SQR(state_.fence_2_xy))
    {
        state_.out_of_fence_2 = true;
    }
    else if (dist_z_sqr > SQR(state_.fence_2_z))
    {
        state_.out_of_fence_2 = true;
    }
    else if (dist_xy_sqr > SQR(state_.fence_1_xy))
    {
        state_.out_of_fence_1 = true;
    }
    else if (dist_z_sqr > SQR(state_.fence_1_z))
    {
        state_.out_of_fence_1 = true;
    }
}

void Position_estimation::calibrate_barometer()
{
    barometer_bias_ = barometer_.altitude_gf() - (- local_position_[Z] + origin().altitude);
    is_barometer_calibrated_ = true;
}
