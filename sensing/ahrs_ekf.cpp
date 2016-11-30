/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
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
 * \file ahrs_ekf.cpp
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 * \author Matthew Douglas
 *
 * \brief Extended Kalman Filter attitude estimation, mixing accelerometer and magnetometer
 * x[0] : bias_x
 * x[1] : bias_y
 * x[2] : bias_z
 * x[3] : q0
 * x[4] : q1
 * x[5] : q2
 * x[6] : q3
 *
 ******************************************************************************/

#include "sensing/ahrs_ekf.hpp"
#include "hal/common/time_keeper.hpp"
#include "util/coord_conventions.hpp"
#include "util/constants.hpp"
#include "util/print_util.hpp"

extern "C"
{
#include "util/maths.h"
#include "util/vectors.h"
#include "util/quaternions.h"
}


using namespace mat;


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

AHRS_ekf::AHRS_ekf(const Imu& imu, const AHRS_ekf::conf_t config):
    Kalman<7, 0, 3>(),
    config_(config),
    imu_(imu),
    attitude_(quat_t{1.0f, {0.0f, 0.0f, 0.0f}}),
    angular_speed_(std::array<float,3>{{0.0f, 0.0f, 0.0f}}),
    linear_acc_(std::array<float,3>{{0.0f, 0.0f, 0.0f}}),
    dt_s_(0.0f),
    last_update_s_(0.0f)
{
    init_kalman();

    internal_state_ = AHRS_INITIALISING;
}


bool AHRS_ekf::update(void)
{
    bool task_return = true;

    // Update time in us
    float now_s    = time_keeper_get_s();

    // Delta t in seconds
    dt_s_          = now_s - last_update_s_;
    last_update_s_ = now_s;

    // To enable changing of R with onboard parameters.
    R_acc_(0,0) = config_.R_acc;
    R_acc_(1,1) = config_.R_acc;
    R_acc_(2,2) = config_.R_acc;

    R_acc_norm_(0,0) = config_.R_acc_norm;
    R_acc_norm_(1,1) = config_.R_acc_norm;
    R_acc_norm_(2,2) = config_.R_acc_norm;

    R_mag_(0,0) = config_.R_mag;
    R_mag_(1,1) = config_.R_mag;
    R_mag_(2,2) = config_.R_mag;

    if (imu_.is_ready())
    {
        internal_state_ = AHRS_READY;

        predict_step();

        if (config_.use_accelerometer)
        {
            update_step_acc();
        }

        if (config_.use_magnetometer)
        {
            update_step_mag();
        }
    }
    else
    {
        internal_state_ = AHRS_LEVELING;

        // Follow accelerometer and magnetometer blindly during IMU calibration
        R_acc_ = Mat<3,3>(0.1f,true);
        R_mag_ = Mat<3,3>(10.0f,true);
        P_ = Mat<7,7>(1.0f,true);

        if (config_.use_accelerometer)
        {
            update_step_acc();
        }

        if (config_.use_magnetometer)
        {
            update_step_mag();
        }

        Mat<7,1> state_previous = x_;

        // Re-init kalman to keep covariance high: ie we follow the raw accelero and magneto without
        // believing them.
        // This allows the filter to re-converge quickly after an imu calibration
        init_kalman();
        for (int i = 3; i < 7; ++i)
        {
            x_(i,0) = state_previous(i,0);
        }
    }

    // Attitude
    attitude_.s = x_(3,0);
    attitude_.v[0] = x_(4,0);
    attitude_.v[1] = x_(5,0);
    attitude_.v[2] = x_(6,0);

    // Angular speed
    angular_speed_[X] = imu_.gyro()[X] - x_(0,0);
    angular_speed_[Y] = imu_.gyro()[Y] - x_(1,0);
    angular_speed_[Z] = imu_.gyro()[Z] - x_(2,0);

    // Linear acceleration
    float up[3] = {0.0f, 0.0f, -1.0f};
    float up_bf[3];
    quaternions_rotate_vector(quaternions_inverse(attitude_), up, up_bf);
    linear_acc_[X] = 9.81f * (imu_.acc()[X] - up_bf[X]);
    linear_acc_[Y] = 9.81f * (imu_.acc()[Y] - up_bf[Y]);
    linear_acc_[Z] = 9.81f * (imu_.acc()[Z] - up_bf[Z]);

    return task_return;
}

float AHRS_ekf::last_update_s(void) const
{
    return last_update_s_;
}


bool AHRS_ekf::is_healthy(void) const
{
    return (internal_state_ == AHRS_READY);
}


quat_t AHRS_ekf::attitude(void) const
{
    return attitude_;
}


std::array<float,3> AHRS_ekf::angular_speed(void) const
{
    return angular_speed_;
}


std::array<float,3> AHRS_ekf::linear_acceleration(void) const
{
    return linear_acc_;
}



//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void AHRS_ekf::init_kalman(void)
{
    P_ = Mat<7,7>(1.0f,true);

    // Initalisation of the state

    x_(0,0) = 0.0f;
    x_(1,0) = 0.0f;
    x_(2,0) = 0.0f;
    x_(3,0) = 1.0f;
    x_(4,0) = 0.0f;
    x_(5,0) = 0.0f;
    x_(6,0) = 0.0f;

    R_acc_ = Mat<3,3>(config_.R_acc,true);

    R_mag_ = Mat<3,3>(config_.R_mag,true);
}

void AHRS_ekf::predict_step(void)
{
    float dt    = dt_s_;
    float dt2_2 = dt * dt * 0.5f;
    float dt3_3 = dt * dt * dt / 3.0f;

    float w_x = imu_.gyro()[X];
    float w_z = imu_.gyro()[Z];
    float w_y = imu_.gyro()[Y];

    float w_sqr = w_x*w_x + w_y*w_y + w_z*w_z;

    Mat<7,1> x_k1k1 = x_;

    Mat<7,1> x_kk1;
    // x(k,k-1) = f(x(k-1,k-1),u(k)); // with zero-order Taylor expansion
    x_kk1(0,0) = x_k1k1(0,0);
    x_kk1(1,0) = x_k1k1(1,0);
    x_kk1(2,0) = x_k1k1(2,0);

    x_kk1(3,0) = x_k1k1(3,0) + 0.5f* (-(w_x-x_k1k1(0,0))*x_k1k1(4,0) - (w_y-x_k1k1(1,0))*x_k1k1(5,0) - (w_z-x_k1k1(2,0))*x_k1k1(6,0)) * dt;
    x_kk1(4,0) = x_k1k1(4,0) + 0.5f* ((w_x-x_k1k1(0,0))*x_k1k1(3,0) + (w_z-x_k1k1(2,0))*x_k1k1(5,0) - (w_y-x_k1k1(1,0))*x_k1k1(6,0)) * dt;
    x_kk1(5,0) = x_k1k1(5,0) + 0.5f* ((w_y-x_k1k1(1,0))*x_k1k1(3,0) - (w_z-x_k1k1(2,0))*x_k1k1(4,0) + (w_x-x_k1k1(0,0))*x_k1k1(6,0)) * dt;
    x_kk1(6,0) = x_k1k1(6,0) + 0.5f* ((w_z-x_k1k1(2,0))*x_k1k1(3,0) + (w_y-x_k1k1(1,0))*x_k1k1(4,0) - (w_x-x_k1k1(0,0))*x_k1k1(5,0)) * dt;

    // F_(k,k-1) = I + jacobian(x(k-1),u(k))*dt;
    F_(0,0) = 1.0f;
    F_(1,1) = 1.0f;
    F_(2,2) = 1.0f;

    F_(3,0) = x_k1k1(4,0) * dt;
    F_(3,1) = x_k1k1(5,0) * dt;
    F_(3,2) = x_k1k1(6,0) * dt;
    F_(3,3) = 1.0f; // 1.0f + 0.0f;
    F_(3,4) = -(w_x-x_k1k1(0,0)) * dt;
    F_(3,5) = -(w_y-x_k1k1(1,0)) * dt;
    F_(3,6) = -(w_z-x_k1k1(2,0)) * dt;

    F_(4,0) = -x_k1k1(3,0) * dt;
    F_(4,1) = x_k1k1(6,0) * dt;
    F_(4,2) = -x_k1k1(5,0) * dt;
    F_(4,3) = (w_x-x_k1k1(0,0)) * dt;
    F_(4,4) = 1.0f; // 1.0f + 0.0f;
    F_(4,5) = (w_z-x_k1k1(2,0)) * dt;
    F_(4,6) = -(w_y-x_k1k1(1,0)) * dt;

    F_(5,0) = -x_k1k1(6,0) * dt;
    F_(5,1) = -x_k1k1(3,0) * dt;
    F_(5,2) = x_k1k1(4,0) * dt;
    F_(5,3) = (w_y-x_k1k1(1,0)) * dt;
    F_(5,4) = -(w_z-x_k1k1(2,0)) * dt;
    F_(5,5) = 1.0f; // 1.0f + 0.0f;
    F_(5,6) = (w_x-x_k1k1(0,0)) * dt;

    F_(6,0) = x_k1k1(5,0) * dt;
    F_(6,1) = -x_k1k1(4,0) * dt;
    F_(6,2) = -x_k1k1(3,0) * dt;
    F_(6,3) = (w_z-x_k1k1(2,0)) * dt;
    F_(6,4) = (w_y-x_k1k1(1,0)) * dt;
    F_(6,5) = -(w_x-x_k1k1(0,0)) * dt;
    F_(6,6) = 1.0f; // 1.0f + 0.0f;

    // Q(k) = cov(del_w * del_w^T)

    Q_(0,0) = config_.sigma_w_sqr * dt;
    Q_(0,1) = 0.0f;
    Q_(0,2) = 0.0f;
    Q_(0,3) = x_kk1[4] * dt2_2 * config_.sigma_w_sqr;
    Q_(0,4) = -x_kk1[3] * dt2_2 * config_.sigma_w_sqr;
    Q_(0,5) = -x_kk1[6] * dt2_2 * config_.sigma_w_sqr;
    Q_(0,6) = x_kk1[5] * dt2_2 * config_.sigma_w_sqr;

    Q_(1,0) = 0.0f;
    Q_(1,1) = config_.sigma_w_sqr * dt;
    Q_(1,2) = 0.0f;
    Q_(1,3) = x_kk1[5] * dt2_2 * config_.sigma_w_sqr;
    Q_(1,4) = x_kk1[6] * dt2_2 * config_.sigma_w_sqr;
    Q_(1,5) = -x_kk1[3] * dt2_2 * config_.sigma_w_sqr;
    Q_(1,6) = -x_kk1[4] * dt2_2 * config_.sigma_w_sqr;

    Q_(2,0) = 0.0f;
    Q_(2,1) = 0.0f;
    Q_(2,2) = config_.sigma_w_sqr * dt;
    Q_(2,3) = x_kk1[6] * dt2_2 * config_.sigma_w_sqr;
    Q_(2,4) = -x_kk1[5] * dt2_2 * config_.sigma_w_sqr;
    Q_(2,5) = x_kk1[4] * dt2_2 * config_.sigma_w_sqr;
    Q_(2,6) = -x_kk1[3] * dt2_2 * config_.sigma_w_sqr;

    Q_(3,0) = x_kk1[4] * dt2_2 * config_.sigma_w_sqr;
    Q_(3,1) = x_kk1[5] * dt2_2 * config_.sigma_w_sqr;
    Q_(3,2) = x_kk1[6] * dt2_2 * config_.sigma_w_sqr;
    Q_(3,3) = config_.sigma_r_sqr * dt + w_sqr * config_.sigma_r_sqr * dt3_3 + (x_kk1[4]*x_kk1[4]+x_kk1[5]*x_kk1[5]+x_kk1[6]*x_kk1[6])*config_.sigma_w_sqr*dt3_3;
    Q_(3,4) = -x_kk1[3] * x_kk1[4] * dt3_3 * config_.sigma_w_sqr;
    Q_(3,5) = -x_kk1[3] * x_kk1[5] * dt3_3 * config_.sigma_w_sqr;
    Q_(3,6) = -x_kk1[3] * x_kk1[6] * dt3_3 * config_.sigma_w_sqr;

    Q_(4,0) = -x_kk1[3] * dt2_2 * config_.sigma_w_sqr;
    Q_(4,1) = x_kk1[6] * dt2_2 * config_.sigma_w_sqr;
    Q_(4,2) = -x_kk1[5] * dt2_2 * config_.sigma_w_sqr;
    Q_(4,3) = -x_kk1[4] * x_kk1[3] * dt3_3 * config_.sigma_w_sqr;
    Q_(4,4) = config_.sigma_r_sqr * dt + w_sqr * config_.sigma_r_sqr * dt3_3 + (x_kk1[3]*x_kk1[3]+x_kk1[5]*x_kk1[5]+x_kk1[6]*x_kk1[6])*config_.sigma_w_sqr*dt3_3;
    Q_(4,5) = -x_kk1[4] * x_kk1[5] * dt3_3 * config_.sigma_w_sqr;
    Q_(4,6) = -x_kk1[4] * x_kk1[6] * dt3_3 * config_.sigma_w_sqr;

    Q_(5,0) = -x_kk1[6] * dt2_2 * config_.sigma_w_sqr;
    Q_(5,1) = -x_kk1[3] * dt2_2 * config_.sigma_w_sqr;
    Q_(5,2) = x_kk1[4] * dt2_2 * config_.sigma_w_sqr;
    Q_(5,3) = -x_kk1[5] * x_kk1[3] * dt3_3 * config_.sigma_w_sqr;
    Q_(5,4) = -x_kk1[5] * x_kk1[4] * dt3_3 * config_.sigma_w_sqr;
    Q_(5,5) = config_.sigma_r_sqr * dt + w_sqr * config_.sigma_r_sqr * dt3_3 + (x_kk1[3]*x_kk1[3]+x_kk1[4]*x_kk1[4]+x_kk1[6]*x_kk1[6])*config_.sigma_w_sqr*dt3_3;
    Q_(5,6) = -x_kk1[5] * x_kk1[6] * dt3_3 * config_.sigma_w_sqr;

    Q_(6,0) = x_kk1[5] * dt2_2 * config_.sigma_w_sqr;
    Q_(6,1) = -x_kk1[4] * dt2_2 * config_.sigma_w_sqr;
    Q_(6,2) = -x_kk1[3] * dt2_2 * config_.sigma_w_sqr;
    Q_(6,3) = -x_kk1[6] * x_kk1[3] * dt3_3 * config_.sigma_w_sqr;
    Q_(6,4) = -x_kk1[6] * x_kk1[4] * dt3_3 * config_.sigma_w_sqr;
    Q_(6,5) = -x_kk1[6] * x_kk1[5] * dt3_3 * config_.sigma_w_sqr;
    Q_(6,6) = config_.sigma_r_sqr * dt + w_sqr * config_.sigma_r_sqr * dt3_3 + (x_kk1[3]*x_kk1[3]+x_kk1[4]*x_kk1[4]+x_kk1[5]*x_kk1[5])*config_.sigma_w_sqr*dt3_3;

    // P_(k,k-1) = F_(k)*P_(k-1,k-1)*F_(k)' + Q_(k)
    P_ = (F_ % P_ % F_.transpose()) + Q_;

    quat_t quat;
    quat.s = x_kk1(3,0);
    quat.v[0] = x_kk1(4,0);
    quat.v[1] = x_kk1(5,0);
    quat.v[2] = x_kk1(6,0);

    quat = quaternions_normalise(quat);
    x_ = x_kk1;
    x_(3,0) = quat.s;
    x_(4,0) = quat.v[0];
    x_(5,0) = quat.v[1];
    x_(6,0) = quat.v[2];
}

void AHRS_ekf::update_step_acc(void)
{
    uint16_t i;

    Mat<7,1> x_kk1 = x_;

    Mat<3,1> z_acc;
    for (i = 0; i < 3; ++i)
    {
        z_acc(i,0) = imu_.acc()[i];
    }

    float acc_z_global = -1.0f;

    // h_acc(x(k,k-1))
    // These equations come from rotating the gravity vector to the local frame
    Mat<3,1> h_acc_xkk1;
    h_acc_xkk1(0,0) = 2.0f*(-x_kk1(3,0)*x_kk1(5,0) + x_kk1(4,0)*x_kk1(6,0)) * acc_z_global;
    h_acc_xkk1(1,0) = 2.0f*(x_kk1(3,0)*x_kk1(4,0) + x_kk1(5,0)*x_kk1(6,0)) * acc_z_global;
    h_acc_xkk1(2,0) = (1.0f - 2.0f*(x_kk1(4,0)*x_kk1(4,0) + x_kk1(5,0)*x_kk1(5,0))) * acc_z_global;

    // H_acc(k) = jacobian(h_acc(x(k,k-1)))
    Mat<3,7> H_acc_k;

    H_acc_k(0,3) = -2.0f * x_kk1(5,0) * acc_z_global;
    H_acc_k(0,4) = 2.0f * x_kk1(6,0) * acc_z_global;
    H_acc_k(0,5) = -2.0f * x_kk1(3,0) * acc_z_global;
    H_acc_k(0,6) = 2.0f * x_kk1(4,0) * acc_z_global;

    H_acc_k(1,3) = 2.0f * x_kk1(4,0) * acc_z_global;
    H_acc_k(1,4) = 2.0f * x_kk1(3,0) * acc_z_global;
    H_acc_k(1,5) = 2.0f * x_kk1(6,0) * acc_z_global;
    H_acc_k(1,6) = 2.0f * x_kk1(5,0) * acc_z_global;

    H_acc_k(2,3) = 0.0f;
    H_acc_k(2,4) = -4.0f * x_kk1(4,0) * acc_z_global;
    H_acc_k(2,5) = -4.0f * x_kk1(5,0) * acc_z_global;
    H_acc_k(2,6) = 0.0f;

    // Innovation y(k) = z(k) - h(x(k,k-1))
    Mat<3,1> yk_acc = z_acc - h_acc_xkk1;

    // Innovation covariance S(k) = H(k) * P_(k,k-1) * H(k)' + R
    Mat<3,3> Sk_acc = (H_acc_k % P_ % H_acc_k.transpose()) + R_acc_ + R_acc_norm_ * maths_f_abs(1.0f - vectors_norm(imu_.acc().data()));

    // Kalman gain: K(k) = P_(k,k-1) * H(k)' * S(k)^-1
    Mat<3,3> Sk_inv;
    op::inverse(Sk_acc, Sk_inv);
    Mat<7,3> K_acc = P_ % (H_acc_k.transpose() % Sk_inv);

    // Updated state estimate: x(k,k) = x(k,k-1) + K(k)*y_k
    Mat<7,1> x_kk = x_kk1 + (K_acc % yk_acc);

    quat_t quat;
    quat.s = x_kk(3,0);
    quat.v[0] = x_kk(4,0);
    quat.v[1] = x_kk(5,0);
    quat.v[2] = x_kk(6,0);

    quat = quaternions_normalise(quat);
    x_ = x_kk;
    x_(3,0) = quat.s;
    x_(4,0) = quat.v[0];
    x_(5,0) = quat.v[1];
    x_(6,0) = quat.v[2];

    // Update covariance estimate
    P_ = (I_ - (K_acc % H_acc_k)) % P_;
}

void AHRS_ekf::update_step_mag(void)
{
    uint16_t i;

    Mat<7,1> x_kk1 = x_;

    std::array<float, 3> mag_global = imu_.magnetic_north();

    Mat<3,1> z_mag;
    for (i = 0; i < 3; ++i)
    {
        z_mag(i,0) = imu_.mag()[i];
    }

    // h_mag(x(k,k-1))
    Mat<3,1> h_mag_xkk1;
    h_mag_xkk1(0,0) = (1.0f + 2.0f*(-x_kk1(5,0)*x_kk1(5,0) - x_kk1(6,0)*x_kk1(6,0)))* mag_global[0] +         2.0f*( x_kk1(3,0)*x_kk1(6,0) + x_kk1(4,0)*x_kk1(5,0)) * mag_global[1] +         2.0f*( x_kk1(4,0)*x_kk1(6,0) - x_kk1(3,0)*x_kk1(5,0)) * mag_global[2];
    h_mag_xkk1(1,0) =         2.0f*( x_kk1(4,0)*x_kk1(5,0) - x_kk1(3,0)*x_kk1(6,0)) * mag_global[0] + (1.0f + 2.0f*(-x_kk1(4,0)*x_kk1(4,0) - x_kk1(6,0)*x_kk1(6,0)))* mag_global[1] +         2.0f*( x_kk1(5,0)*x_kk1(6,0) + x_kk1(3,0)*x_kk1(4,0)) * mag_global[2];
    h_mag_xkk1(2,0) =         2.0f*( x_kk1(4,0)*x_kk1(6,0) + x_kk1(3,0)*x_kk1(5,0)) * mag_global[0] +         2.0f*( x_kk1(5,0)*x_kk1(6,0) - x_kk1(3,0)*x_kk1(4,0)) * mag_global[1] + (1.0f + 2.0f*(-x_kk1(4,0)*x_kk1(4,0) - x_kk1(5,0)*x_kk1(5,0)))* mag_global[2];

    Mat<3,7> H_mag_k;

    H_mag_k(0,3) =                                  2.0f*x_kk1(6,0)*mag_global[1] - 2.0f*x_kk1(5,0)*mag_global[2];
    H_mag_k(0,4) =                                  2.0f*x_kk1(5,0)*mag_global[1] + 2.0f*x_kk1(6,0)*mag_global[2];
    H_mag_k(0,5) = -4.0f*x_kk1(5,0)*mag_global[0] + 2.0f*x_kk1(4,0)*mag_global[1] - 2.0f*x_kk1(3,0)*mag_global[2];
    H_mag_k(0,6) = -4.0f*x_kk1(6,0)*mag_global[0] + 2.0f*x_kk1(3,0)*mag_global[1] + 2.0f*x_kk1(4,0)*mag_global[2];

    H_mag_k(1,3) = -2.0f*x_kk1(6,0)*mag_global[0]                                 + 2.0f*x_kk1(4,0)*mag_global[2];
    H_mag_k(1,4) =  2.0f*x_kk1(5,0)*mag_global[0] - 4.0f*x_kk1(4,0)*mag_global[1] + 2.0f*x_kk1(3,0)*mag_global[2];
    H_mag_k(1,5) =  2.0f*x_kk1(4,0)*mag_global[0]                                 + 2.0f*x_kk1(6,0)*mag_global[2];
    H_mag_k(1,6) = -2.0f*x_kk1(3,0)*mag_global[0] - 4.0f*x_kk1(6,0)*mag_global[1] + 2.0f*x_kk1(5,0)*mag_global[2];

    H_mag_k(2,3) =  2.0f*x_kk1(5,0)*mag_global[0] - 2.0f*x_kk1(4,0)*mag_global[1];
    H_mag_k(2,4) =  2.0f*x_kk1(6,0)*mag_global[0] - 2.0f*x_kk1(3,0)*mag_global[1] - 4.0f*x_kk1(4,0)*mag_global[2];
    H_mag_k(2,5) =  2.0f*x_kk1(3,0)*mag_global[0] + 2.0f*x_kk1(6,0)*mag_global[1] - 4.0f*x_kk1(5,0)*mag_global[2];
    H_mag_k(2,6) =  2.0f*x_kk1(4,0)*mag_global[0] + 2.0f*x_kk1(5,0)*mag_global[1];

    // Innovation y(k) = z(k) - h(x(k,k-1))
    Mat<3,1> yk_mag = z_mag - h_mag_xkk1;

    // Innovation covariance S(k) = H(k) * P_(k,k-1) * H(k)' + R
    Mat<3,3> Sk_mag = (H_mag_k % P_ % H_mag_k.transpose()) + R_mag_;

    // Kalman gain: K(k) = P_(k,k-1) * H(k)' * S(k)^-1
    Mat<3,3> Sk_inv;
    op::inverse(Sk_mag, Sk_inv);
    Mat<7,3> K_mag = P_ % (H_mag_k.transpose() % Sk_inv);

    // Updated state estimate: x(k,k) = x(k,k-1) + K(k)*y_k
    Mat<7,1> x_kk = x_kk1 + (K_mag % yk_mag);
    //Mat<7,1> x_kk = x_kk1;

    quat_t quat;
    quat.s = x_kk(3,0);
    quat.v[0] = x_kk(4,0);
    quat.v[1] = x_kk(5,0);
    quat.v[2] = x_kk(6,0);

    quat = quaternions_normalise(quat);
    x_ = x_kk;
    x_(3,0) = quat.s;
    x_(4,0) = quat.v[0];
    x_(5,0) = quat.v[1];
    x_(6,0) = quat.v[2];

    // Update covariance estimate
    P_ = (I_ - (K_mag % H_mag_k)) % P_;
}
