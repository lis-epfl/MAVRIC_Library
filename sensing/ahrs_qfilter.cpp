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
 * \file qfilter.cpp
 *
 * \author MAV'RIC Team
 * \author Felix Schill
 *
 * \brief This file implements a complementary filter for the attitude estimation
 *
 ******************************************************************************/


#include "hal/common/time_keeper.hpp"
#include "sensing/ahrs_qfilter.hpp"
#include "util/coord_conventions.hpp"
#include "util/constants.hpp"
#include "util/print_util.hpp"

extern "C"
{
#include <math.h>
#include "util/maths.h"
}


AHRS_qfilter::AHRS_qfilter(const Imu& imu, const conf_t& config):
    imu_(imu),
    attitude_(quat_t{1.0f, {0.0f, 0.0f, 0.0f}}),
    angular_speed_{{0.0f, 0.0f, 0.0f}},
    linear_acc_{{0.0f, 0.0f, 0.0f}},
    last_update_s_(0.0f)
{
    //init qfilter gains according to provided configuration
    kp_ = config.kp;
    ki_ = config.ki;
    kp_mag_ = config.kp_mag;
    ki_mag_ = config.ki_mag;

    gyro_bias_ = std::array<float, 3> {{0.0f, 0.0f, 0.0f}};
}


bool AHRS_qfilter::update(void)
{
    float  omc[3], omc_mag[3] , tmp[3], snorm, norm, s_acc_norm, acc_norm, s_mag_norm, mag_norm;
    quat_t qed, qtmp1, up, up_bf;
    quat_t mag_global, mag_corrected_local;

    quat_t front_vec_global = {};
    front_vec_global.s    = 0.0f;
    front_vec_global.v[0] = 1.0f;
    front_vec_global.v[1] = 0.0f;
    front_vec_global.v[2] = 0.0f;

    float kp     = kp_;
    float kp_mag = kp_mag_;
    float ki     = ki_;
    float ki_mag = ki_mag_;

    // Update time in us
    float now_s    = time_keeper_get_s();

    // Delta t in seconds
    float dt_s     = (float)(now_s - last_update_s_);

    // Write to ahrs structure
    last_update_s_ = now_s;

    // up_bf = qe^-1 *(0,0,0,-1) * qe
    up.s = 0; up.v[X] = UPVECTOR_X; up.v[Y] = UPVECTOR_Y; up.v[Z] = UPVECTOR_Z;
    up_bf = quaternions_global_to_local(attitude_, up);

    // Get data from IMU
    std::array<float, 3> acc  = imu_.acc();
    std::array<float, 3> gyro = imu_.gyro();
    std::array<float, 3> mag  = imu_.mag();

    // Remove estimated bias from gyros
    gyro[X] -= gyro_bias_[X];
    gyro[Y] -= gyro_bias_[Y];
    gyro[Z] -= gyro_bias_[Z];

    // calculate norm of acceleration vector
    s_acc_norm =  acc[X] * acc[X] + acc[Y] * acc[Y] + acc[Z] * acc[Z];
    if ((s_acc_norm > 0.7f * 0.7f) && (s_acc_norm < 1.3f * 1.3f))
    {
        // approximate square root by running 2 iterations of newton method
        acc_norm = maths_fast_sqrt(s_acc_norm);

        tmp[X] = acc[X] / acc_norm;
        tmp[Y] = acc[Y] / acc_norm;
        tmp[Z] = acc[Z] / acc_norm;

        // omc = a x up_bf.v
        CROSS(tmp, up_bf.v, omc);
    }
    else
    {
        omc[X] = 0;
        omc[Y] = 0;
        omc[Z] = 0;
    }

    // Heading computation
    // transfer
    qtmp1 = quat_t{0.0f, {mag[X], mag[Y], mag[Z]}};
    mag_global = quaternions_local_to_global(attitude_, qtmp1);

    // calculate norm of compass vector
    //s_mag_norm = SQR(mag_global.v[X]) + SQR(mag_global.v[Y]) + SQR(mag_global.v[Z]);
    s_mag_norm = SQR(mag_global.v[X]) + SQR(mag_global.v[Y]);

    if ((s_mag_norm > 0.004f * 0.004f) && (s_mag_norm < 1.8f * 1.8f))
    {
        mag_norm = maths_fast_sqrt(s_mag_norm);

        mag_global.v[X] /= mag_norm;
        mag_global.v[Y] /= mag_norm;
        mag_global.v[Z] = 0.0f;   // set z component in global frame to 0

        // transfer magneto vector back to body frame
        quat_t north_vec = quaternions_global_to_local(attitude_, front_vec_global);

        mag_corrected_local = quaternions_global_to_local(attitude_, mag_global);

        // omc = a x up_bf.v
        CROSS(mag_corrected_local.v, north_vec.v,  omc_mag);

    }
    else
    {
        omc_mag[X] = 0;
        omc_mag[Y] = 0;
        omc_mag[Z] = 0;
    }


    // get error correction gains depending on mode
    switch (internal_state_)
    {
        case AHRS_INITIALISING:
            time_s_ = time_keeper_get_s();
            internal_state_ = AHRS_LEVELING;
        case AHRS_LEVELING:
            kp = kp_ * 10.0f;
            kp_mag = kp_mag_ * 10.0f;

            ki = 0.0f * ki_;
            ki_mag = 0.0f * ki_mag_;

            if ((time_keeper_get_s() - time_s_) > 8.0f)
            {
                time_s_ = time_keeper_get_s();
                internal_state_ = AHRS_CONVERGING;
                print_util_dbg_print("End of AHRS attitude initialization.\r\n");
            }
            break;

        case AHRS_CONVERGING:
            kp = kp_;
            kp_mag = kp_mag_;

            //ki = ki_ * 1.5f;
            //ki_mag = ki_mag_ * 1.5f;

            if (imu_.is_ready())
            {
                internal_state_ = AHRS_READY;
                print_util_dbg_print("End of AHRS leveling.\r\n");
            }
            break;

        case AHRS_READY:
            kp = kp_;
            kp_mag = kp_mag_;
            ki = ki_;
            ki_mag = ki_mag_;
            break;
    }

    // apply error correction with appropriate gains for accelerometer and compass

    for (uint8_t i = 0; i < 3; i++)
    {
        qtmp1.v[i] = 0.5f * (gyro[i] + kp * omc[i] + kp_mag * omc_mag[i]);
    }
    qtmp1.s = 0;

    // apply step rotation with corrections
    qed = quaternions_multiply(attitude_, qtmp1);

    // TODO: correct this formulas!
    attitude_.s = attitude_.s + qed.s * dt_s;
    attitude_.v[X] += qed.v[X] * dt_s;
    attitude_.v[Y] += qed.v[Y] * dt_s;
    attitude_.v[Z] += qed.v[Z] * dt_s;

    snorm = attitude_.s * attitude_.s + attitude_.v[X] * attitude_.v[X] + attitude_.v[Y] * attitude_.v[Y] + attitude_.v[Z] * attitude_.v[Z];
    if (snorm < 0.0001f)
    {
        norm = 0.01f;
    }
    else
    {
        // approximate square root by running 2 iterations of newton method
        norm = maths_fast_sqrt(snorm);
    }
    attitude_.s /= norm;
    attitude_.v[X] /= norm;
    attitude_.v[Y] /= norm;
    attitude_.v[Z] /= norm;

    // bias estimate update
    gyro_bias_[X] += - dt_s * ki * omc[X];
    gyro_bias_[Y] += - dt_s * ki * omc[Y];
    gyro_bias_[Z] += - dt_s * ki * omc[Z];

    gyro_bias_[X] += - dt_s * ki_mag * omc_mag[X];
    gyro_bias_[Y] += - dt_s * ki_mag * omc_mag[Y];
    gyro_bias_[Z] += - dt_s * ki_mag * omc_mag[Z];

    // Update linear acceleration
    linear_acc_[X] = 9.81f * (acc[X] - up_bf.v[X]) ;
    linear_acc_[Y] = 9.81f * (acc[Y] - up_bf.v[Y]) ;
    linear_acc_[Z] = 9.81f * (acc[Z] - up_bf.v[Z]) ;

    // Update angular_speed.
    angular_speed_[X] = gyro[X];
    angular_speed_[Y] = gyro[Y];
    angular_speed_[Z] = gyro[Z];

    return true;
}


float AHRS_qfilter::last_update_s(void) const
{
    return last_update_s_;
}


bool AHRS_qfilter::is_healthy(void) const
{
    return (internal_state_ == AHRS_READY);
}


quat_t AHRS_qfilter::attitude(void) const
{
    return attitude_;
}


std::array<float,3> AHRS_qfilter::angular_speed(void) const
{
    return angular_speed_;
}


std::array<float,3> AHRS_qfilter::linear_acceleration(void) const
{
    return linear_acc_;
}
