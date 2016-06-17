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

extern "C"
{
#include "util/constants.h"
#include "util/print_util.h"
#include "util/maths.h"
#include "util/vectors.h"
#include "util/quaternions.h"
#include "util/coord_conventions.h"
}


using namespace mat;

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void Ahrs_ekf::init_kalman(void)
{
    P_ = Mat<7,7>(1.0f,true);

    // Initalisation of the state

    x_state_(0,0) = 0.0f;
    x_state_(1,0) = 0.0f;
    x_state_(2,0) = 0.0f;
    x_state_(3,0) = 1.0f;
    x_state_(4,0) = 0.0f;
    x_state_(5,0) = 0.0f;
    x_state_(6,0) = 0.0f;

    R_acc_ = Mat<3,3>(config_.R_acc,true);

    R_mag_ = Mat<3,3>(config_.R_mag,true);
}

void Ahrs_ekf::predict_step(void)
{
    float dt = ahrs_.dt_s;

    float dt2_2 = dt * dt * 0.5f;
    float dt3_3 = dt * dt * dt / 3.0f;

    float w_x = imu_.gyro()[X];
    float w_z = imu_.gyro()[Z];
    float w_y = imu_.gyro()[Y];

    float w_sqr = w_x*w_x + w_y*w_y + w_z*w_z;

    Mat<7,1> x_k1k1 = x_state_;

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
    x_state_ = x_kk1;
    x_state_(3,0) = quat.s;
    x_state_(4,0) = quat.v[0];
    x_state_(5,0) = quat.v[1];
    x_state_(6,0) = quat.v[2];
}

void Ahrs_ekf::update_step_acc(void)
{
    uint16_t i;

    Mat<7,1> x_kk1 = x_state_;

    Mat<3,1> z_acc;
    for (i = 0; i < 3; ++i)
    {
        z_acc(i,0) = imu_.acc()[i];
    }

    float acc_z_global = -1.0f;

    // h_acc(x(k,k-1))
    Mat<3,1> h_acc_xkk1;
    h_acc_xkk1(0,0) = 2.0f*(x_kk1(3,0)*x_kk1(5,0) + x_kk1(4,0)*x_kk1(6,0)) * acc_z_global;
    h_acc_xkk1(1,0) = 2.0f*(-x_kk1(3,0)*x_kk1(4,0) + x_kk1(5,0)*x_kk1(6,0)) * acc_z_global;
    h_acc_xkk1(2,0) = (1.0f - 2.0f*(x_kk1(4,0)*x_kk1(4,0) + x_kk1(5,0)*x_kk1(5,0))) * acc_z_global;

    // H_acc(k) = jacobian(h_acc(x(k,k-1)))
    Mat<3,7> H_acc_k;

    H_acc_k(0,3) = 2.0f * x_kk1(5,0) * acc_z_global;
    H_acc_k(0,4) = 2.0f * x_kk1(6,0) * acc_z_global;
    H_acc_k(0,5) = 2.0f * x_kk1(3,0) * acc_z_global;
    H_acc_k(0,6) = 2.0f * x_kk1(4,0) * acc_z_global;

    H_acc_k(1,3) = -2.0f * x_kk1(4,0) * acc_z_global;
    H_acc_k(1,4) = -2.0f * x_kk1(3,0) * acc_z_global;
    H_acc_k(1,5) = 2.0f * x_kk1(6,0) * acc_z_global;
    H_acc_k(1,6) = 2.0f * x_kk1(5,0) * acc_z_global;

    H_acc_k(2,3) = 0.0f;
    H_acc_k(2,4) = -4.0f * x_kk1(4,0) * acc_z_global;
    H_acc_k(2,5) = -4.0f * x_kk1(5,0) * acc_z_global;
    H_acc_k(2,6) = 0.0f;

    // Innovation y(k) = z(k) - h(x(k,k-1))
    Mat<3,1> yk_acc = z_acc - h_acc_xkk1;

    float acc[3];
    acc[0] = imu_.acc()[0];
    acc[1] = imu_.acc()[1];
    acc[2] = imu_.acc()[2];
    float acc_norm = vectors_norm(acc);
    float acc_norm_diff = maths_f_abs(1.0f-acc_norm);
    float noise = 1.0f - maths_center_window_4(config_.acc_multi_noise*acc_norm_diff);

    // Innovation covariance S(k) = H(k) * P_(k,k-1) * H(k)' + R
    Mat<3,3> Sk_acc = (H_acc_k % P_ % H_acc_k.transpose()) + R_acc_ + Mat<3,3>(noise*config_.acc_norm_noise,true);

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
    x_state_ = x_kk;
    x_state_(3,0) = quat.s;
    x_state_(4,0) = quat.v[0];
    x_state_(5,0) = quat.v[1];
    x_state_(6,0) = quat.v[2];

    // Update covariance estimate
    P_ = (Id_ - (K_acc % H_acc_k)) % P_;

}

void Ahrs_ekf::update_step_mag(void)
{
    uint16_t i;

    Mat<7,1> x_kk1 = x_state_;

    std::array<float, 3> mag_global = imu_.magnetic_north();

    Mat<3,1> z_mag;
    for (i = 0; i < 3; ++i)
    {
        z_mag(i,0) = imu_.mag()[i];
    }

    // h_mag(x(k,k-1))
    Mat<3,1> h_mag_xkk1;
    h_mag_xkk1(0,0) = (1.0f - 2.0f*(x_kk1(5,0)*x_kk1(5,0) + x_kk1(6,0)*x_kk1(6,0)))*mag_global[0] + 2.0f*(x_kk1(4,0)*x_kk1(6,0) - x_kk1(3,0)*x_kk1(5,0))*mag_global[2];
    h_mag_xkk1(1,0) = 2.0f*(x_kk1(4,0)*x_kk1(5,0) - x_kk1(3,0)*x_kk1(6,0))*mag_global[0] + 2.0f*(x_kk1(5,0)*x_kk1(6,0) + x_kk1(3,0)*x_kk1(4,0))*mag_global[2];
    h_mag_xkk1(2,0) = 2.0f*(x_kk1(4,0)*x_kk1(6,0) - x_kk1(3,0)*x_kk1(5,0))*mag_global[0] + (1.0f - 2.0f*(x_kk1(4,0)*x_kk1(4,0) + x_kk1(5,0)*x_kk1(5,0)))*mag_global[2];

    Mat<3,7> H_mag_k;

    H_mag_k(0,3) = -2.0f * x_kk1(5,0) * mag_global[2];
    H_mag_k(0,4) = 2.0f * x_kk1(6,0) * mag_global[2];
    H_mag_k(0,5) = -4.0f * x_kk1(5,0) * mag_global[0] - 2.0f * x_kk1(3,0) * mag_global[2];
    H_mag_k(0,6) = -4.0f * x_kk1(6,0) * mag_global[0] + 2.0f * x_kk1(4,0) * mag_global[2];

    H_mag_k(1,3) = -2.0f * x_kk1(6,0) * mag_global[0] + 2.0f * x_kk1(4,0) * mag_global[2];
    H_mag_k(1,4) = 2.0f * x_kk1(5,0) * mag_global[0] + 2.0f * x_kk1(3,0) * mag_global[2];
    H_mag_k(1,5) = 2.0f * x_kk1(4,0) * mag_global[0] + 2.0f * x_kk1(6,0) * mag_global[2];
    H_mag_k(1,6) = -2.0f * x_kk1(3,0) * mag_global[0] + 2.0f * x_kk1(5,0) * mag_global[2];

    H_mag_k(2,3) = -2.0f * x_kk1(5,0) * mag_global[0];
    H_mag_k(2,4) = 2.0f * x_kk1(6,0) * mag_global[0] - 4.0f * x_kk1(4,0) * mag_global[2];
    H_mag_k(2,5) = -2.0f * x_kk1(3,0) * mag_global[0] - 4.0f * x_kk1(5,0) * mag_global[2];
    H_mag_k(2,6) = 2.0f * x_kk1(4,0) * mag_global[0];

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
    x_state_ = x_kk;
    x_state_(3,0) = quat.s;
    x_state_(4,0) = quat.v[0];
    x_state_(5,0) = quat.v[1];
    x_state_(6,0) = quat.v[2];

    // Update covariance estimate
    P_ = (Id_ - (K_mag % H_mag_k)) % P_;

}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Ahrs_ekf::Ahrs_ekf(const Imu& imu, ahrs_t& ahrs, const Ahrs_ekf::conf_t config):
    Id_(Mat<7,7>(1.0f,true)),
    config_(config),
    imu_(imu),
    ahrs_(ahrs)
{
    init_kalman();

    ahrs_.internal_state = AHRS_INITIALISING;
}

bool Ahrs_ekf::update(void)
{
    bool task_return = true;

    // Update time in us
    float now_s    = time_keeper_get_s();

    // Delta t in seconds
    float dt_s     = now_s - ahrs_.last_update_s;

    // Write to ahrs structure
    ahrs_.dt_s          = dt_s;
    ahrs_.last_update_s = now_s;

    // To enable changing of R with onboard parameters.
    R_acc_(0,0) = config_.R_acc;
    R_acc_(1,1) = config_.R_acc;
    R_acc_(2,2) = config_.R_acc;

    R_mag_(0,0) = config_.R_mag;
    R_mag_(1,1) = config_.R_mag;
    R_mag_(2,2) = config_.R_mag;

    if (imu_.is_ready())
    {
        ahrs_.internal_state = AHRS_READY;

        predict_step();

        update_step_acc();

        update_step_mag();
    }
    else
    {
        ahrs_.internal_state = AHRS_LEVELING;

        // Follow accelerometer and magnetometer blindly during IMU calibration
        R_acc_ = Mat<3,3>(0.1f,true);
        R_mag_ = Mat<3,3>(10.0f,true);
        P_ = Mat<7,7>(1.0f,true);
        update_step_acc();
        update_step_mag();
        Mat<7,1> state_previous = x_state_;

        // Re-init kalman to keep covariance high: ie we follow the raw accelero and magneto without
        // believing them.
        // This allows the filter to re-converge quickly after an imu calibration
        init_kalman();
        for (int i = 3; i < 7; ++i)
        {
            x_state_(i,0) = state_previous(i,0);
        }
    }

    // Copy result into ahrs structure

    // Attitude
    ahrs_.qe.s = x_state_(3,0);
    ahrs_.qe.v[0] = x_state_(4,0);
    ahrs_.qe.v[1] = x_state_(5,0);
    ahrs_.qe.v[2] = x_state_(6,0);

    // Angular speed
    ahrs_.angular_speed[X] = imu_.gyro()[X] - x_state_(0,0);
    ahrs_.angular_speed[Y] = imu_.gyro()[Y] - x_state_(1,0);
    ahrs_.angular_speed[Z] = imu_.gyro()[Z] - x_state_(2,0);

    // Linear acceleration
    float up[3] = {0.0f, 0.0f, -1.0f};
    float up_bf[3];
    quaternions_rotate_vector(quaternions_inverse(ahrs_.qe), up, up_bf);
    ahrs_.linear_acc[X] = 9.81f * (imu_.acc()[X] - up_bf[X]);
    ahrs_.linear_acc[Y] = 9.81f * (imu_.acc()[Y] - up_bf[Y]);
    ahrs_.linear_acc[Z] = 9.81f * (imu_.acc()[Z] - up_bf[Z]);

    return task_return;
}
