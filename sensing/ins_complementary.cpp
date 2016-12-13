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
 * \file ins_complementary.cpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief This file performs the 3D position estimation, either by direct
 * integration or with correction with the GPS and pos_est->barometer
 *
 ******************************************************************************/


#include "sensing/ins_complementary.hpp"
#include "hal/common/time_keeper.hpp"
#include "util/print_util.hpp"

extern "C"
{
#include "util/maths.h"
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

INS_complementary::INS_complementary(State& state, const Barometer& barometer, const Sonar& sonar, const Gps& gps, const PX4Flow& flow, const AHRS& ahrs, const conf_t config) :
    local_position_(std::array<float,3>{{0.0f, 0.0f, 0.0f}}),
    vel_(std::array<float,3>{{0.0f, 0.0f, 0.0f}}),
    acc_bias_(std::array<float,3>{{0.0f, 0.0f, 0.0f}}),
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


bool INS_complementary::update(void)
{
    // Updat timing
    float now      = time_keeper_get_s();
    dt_s_          = now - last_update_s_;
    last_update_s_ = now;


    if (ahrs_.is_healthy())
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

        return true;
    }
    else
    {
        return false;
    }
}


void INS_complementary::reset_velocity_altitude()
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


float INS_complementary::last_update_s(void) const
{
    return last_update_s_;
}


std::array<float,3> INS_complementary::position_lf(void) const
{
    return local_position_;
}


std::array<float,3> INS_complementary::velocity_lf(void) const
{
    return std::array<float, 3>{{vel_[0], vel_[1], vel_[2]}};
}


float INS_complementary::absolute_altitude(void) const
{
    return (origin().altitude - local_position_[Z]);
}


bool INS_complementary::is_healthy(INS::healthy_t __attribute__((unused)) type) const
{
    return true;
}


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void INS_complementary::integration()
{
    // Get velocity in body frame
    float vel_bf[3];
    quaternions_rotate_vector(quaternions_inverse(ahrs_.attitude()), vel_.data(), vel_bf);

    // Integrate velocity using accelerometer
    for (uint32_t i = 0; i < 3; i++)
    {
        vel_bf[i] += ahrs_.linear_acceleration()[i] * dt_s_;

        if (config_.use_acc_bias)
        {
            vel_bf[i] += acc_bias_[i] * dt_s_;
        }
    }

    // Get new velocity in local frame
    quaternions_rotate_vector(ahrs_.attitude(), vel_bf, vel_.data());

    // Integrate position using estimated velocity
    for (uint32_t i = 0; i < 3; i++)
    {
        local_position_[i] += vel_[i] * dt_s_;
    }
}


void INS_complementary::correction_from_3d_pos(std::array<float,3> error, std::array<float,3> gain)
{
    // Apply error correction to position estimates
    local_position_[X] += gain[X] * error[X] * dt_s_;
    local_position_[Y] += gain[Y] * error[Y] * dt_s_;
    local_position_[Z] += gain[Z] * error[Z] * dt_s_;

    if (config_.use_acc_bias)
    {
        std::array<float,3> error_lf = {{gain[X] * error[X], gain[Y] * error[Y], gain[Z] * error[Z]}};
        std::array<float,3> error_bf;
        quaternions_rotate_vector(quaternions_inverse(ahrs_.attitude()), error_lf.data(), error_bf.data());
        acc_bias_[X] += config_.kp_acc_bias * error_bf[X] * dt_s_;
        acc_bias_[Y] += config_.kp_acc_bias * error_bf[Y] * dt_s_;
        acc_bias_[Z] += config_.kp_acc_bias * error_bf[Z] * dt_s_;
    }
}


void INS_complementary::correction_from_z_pos(float error, float gain)
{
    // Apply error correction to position estimates
    local_position_[Z] += gain * error * dt_s_;

    if (config_.use_acc_bias)
    {
        std::array<float,3> error_lf = {{0.0f, 0.0f, gain * error}};
        std::array<float,3> error_bf;
        quaternions_rotate_vector(quaternions_inverse(ahrs_.attitude()), error_lf.data(), error_bf.data());
        acc_bias_[X] += config_.kp_acc_bias * error_bf[X] * dt_s_;
        acc_bias_[Y] += config_.kp_acc_bias * error_bf[Y] * dt_s_;
        acc_bias_[Z] += config_.kp_acc_bias * error_bf[Z] * dt_s_;
    }
}


void INS_complementary::correction_from_3d_vel(std::array<float,3> error, std::array<float,3> gain)
{
    // Apply error correction to velocity estimates
    vel_[X] += gain[X] * error[X] * dt_s_;
    vel_[Y] += gain[Y] * error[Y] * dt_s_;
    vel_[Z] += gain[Z] * error[Z] * dt_s_;

    if (config_.use_acc_bias)
    {
        std::array<float,3> error_lf = {{gain[X] * error[X], gain[Y] * error[Y], gain[Z] * error[Z]}};
        std::array<float,3> error_bf;
        quaternions_rotate_vector(quaternions_inverse(ahrs_.attitude()), error_lf.data(), error_bf.data());
        acc_bias_[X] += config_.kp_acc_bias * error_bf[X] * SQR(dt_s_);
        acc_bias_[Y] += config_.kp_acc_bias * error_bf[Y] * SQR(dt_s_);
        acc_bias_[Z] += config_.kp_acc_bias * error_bf[Z] * SQR(dt_s_);
    }
}


void INS_complementary::correction_from_z_vel(float error, float gain)
{
    // Apply error correction to velocity estimates
    vel_[Z] += gain * error * dt_s_;

    if (config_.use_acc_bias)
    {
        std::array<float,3> error_lf = {{0.0f, 0.0f, gain * error}};
        std::array<float,3> error_bf;
        quaternions_rotate_vector(quaternions_inverse(ahrs_.attitude()), error_lf.data(), error_bf.data());
        acc_bias_[X] += config_.kp_acc_bias * error_bf[X] * SQR(dt_s_);
        acc_bias_[Y] += config_.kp_acc_bias * error_bf[Y] * SQR(dt_s_);
        acc_bias_[Z] += config_.kp_acc_bias * error_bf[Z] * SQR(dt_s_);
    }
}


void INS_complementary::correction_from_gps_pos(void)
{
    if (gps_.healthy() == true)
    {
        if (last_gps_pos_update_us_ < gps_.last_position_update_us() + config_.timeout_gps_us)
        {
            last_gps_pos_update_us_ = gps_.last_position_update_us();

            // get local position from GPS
            local_position_t gps_pos;
            coord_conventions_global_to_local_position(gps_.position_gf(), origin(), gps_pos);

            // Compute position error and velocity error from gps
            std::array<float,3> pos_error;
            for (uint32_t i = 0; i < 3; i++)
            {
                pos_error[i] = gps_pos[i] - local_position_[i];
            }

            // Get correct gain
            std::array<float,3> gain = {{0.0f, 0.0f, 0.0f}};
            if (gps_.fix() == FIX_RTK)
            {
                gain[X] = config_.kp_gps_XY_pos_rtk;
                gain[Y] = config_.kp_gps_XY_pos_rtk;
                gain[Z] = config_.kp_gps_Z_pos_rtk;
            }
            else if (gps_.fix() == FIX_DGPS)
            {
                gain[X] = config_.kp_gps_XY_pos_dgps;
                gain[Y] = config_.kp_gps_XY_pos_dgps;
                gain[Z] = config_.kp_gps_Z_pos_dgps;
            }
            else if (gps_.fix() == FIX_3D)
            {
                gain[X] = config_.kp_gps_XY_pos;
                gain[Y] = config_.kp_gps_XY_pos;
                gain[Z] = config_.kp_gps_Z_pos;
            }

            // Apply error correction to position estimates
            correction_from_3d_pos(pos_error, gain);
        }
    }
}


void INS_complementary::correction_from_gps_vel(void)
{
    if (gps_.healthy() == true)
    {
        if (last_gps_vel_update_us_ < gps_.last_velocity_update_us() + config_.timeout_gps_us)
        {
            last_gps_vel_update_us_ = gps_.last_velocity_update_us();

            // Compute position error and velocity error from gps
            std::array<float,3> vel_error;
            for (uint32_t i = 0; i < 3; i++)
            {
                vel_error[i] = gps_.velocity_lf()[i] - vel_[i];
            }

            // Get correct gain
            std::array<float,3> gain = {{0.0f, 0.0f, 0.0f}};
            if (gps_.fix() == FIX_RTK)
            {
                gain[X] = config_.kp_gps_XY_vel_rtk;
                gain[Y] = config_.kp_gps_XY_vel_rtk;
                gain[Z] = config_.kp_gps_Z_vel_rtk;
            }
            else if (gps_.fix() == FIX_DGPS)
            {
                gain[X] = config_.kp_gps_XY_vel_dgps;
                gain[Y] = config_.kp_gps_XY_vel_dgps;
                gain[Z] = config_.kp_gps_Z_vel_dgps;
            }
            else if (gps_.fix() == FIX_3D)
            {
                gain[X] = config_.kp_gps_XY_vel;
                gain[Y] = config_.kp_gps_XY_vel;
                gain[Z] = config_.kp_gps_Z_vel;
            }

            // Apply error correction to velocity estimates
            correction_from_3d_vel(vel_error, gain);
        }
    }
}


void INS_complementary::correction_from_barometer(void)
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
    correction_from_z_pos(baro_alt_error, gain_alt);
    correction_from_z_vel(baro_vel_error, gain_vel);
}


void INS_complementary::correction_from_sonar(void)
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
    correction_from_z_pos(sonar_alt_error, config_.kp_sonar_alt * gain_alt);
    correction_from_z_vel(sonar_vel_error, config_.kp_sonar_vel * gain_vel);
}


void INS_complementary::correction_from_flow(void)
{
    std::array<float,3> vel_error = {{0.0f, 0.0f, 0.0f}};
    std::array<float,3> vel_gain   = {{config_.kp_flow_vel, config_.kp_flow_vel, config_.kp_sonar_vel}};
    float alt_error = 0.0f;
    float alt_gain  = config_.kp_sonar_alt;

    if (flow_.healthy())
    {
        if (last_flow_update_us_ < (flow_.last_update_s() * 1e6f) + config_.timeout_flow_us)
        {
            last_flow_update_us_ = (flow_.last_update_s() * 1e6f);

            // Get velocity in NED frame
            float vel_lf[3];
            float vel_bf[3] = {flow_.velocity_x(), flow_.velocity_y(), flow_.velocity_z()};
            quaternions_rotate_vector(coord_conventions_quaternion_from_rpy(0.0f,
                                                                            0.0f,
                                                                            coord_conventions_get_yaw(ahrs_.attitude())),
                                      vel_bf,
                                      vel_lf);

            // Compute errors
            vel_error[0] = vel_lf[X] - vel_[X];
            vel_error[1] = vel_lf[Y] - vel_[Y];
            vel_error[2] = vel_lf[Z] - vel_[Z];
            alt_error    = - flow_.ground_distance() - local_position_[Z];

            // Apply corrections
            correction_from_3d_vel(vel_error, vel_gain);
            correction_from_z_pos(alt_error, alt_gain);
        }
    }

}


void INS_complementary::check_first_gps_fix()
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


void INS_complementary::calibrate_barometer()
{
    barometer_bias_ = barometer_.altitude_gf() - (- local_position_[Z] + origin().altitude);
    is_barometer_calibrated_ = true;
}
