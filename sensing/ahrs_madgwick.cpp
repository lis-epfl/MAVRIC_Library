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
 * \file ahrs_madgwick.cpp
 *
 * \author MAV'RIC Team
 * \author SOH Madgwick
 * \author Julien Lecoeur
 * \author Simon Pyroth
 *
 * \brief Implementation of Madgwick's AHRS algorithms.
 *
 * See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
 *
 * Date         Author          Notes
 * 29/09/2011   SOH Madgwick    Initial release
 * 02/10/2011   SOH Madgwick    Optimised for reduced CPU load
 * 19/02/2012   SOH Madgwick    Magnetometer measurement is normalised
 * 04/02/2014   Julien Lecoeur  Adapt to MAVRIC
 *
 ******************************************************************************/

#include "sensing/ahrs_madgwick.hpp"
#include "hal/common/time_keeper.hpp"
#include "util/constants.hpp"
#include "util/print_util.hpp"

extern "C"
{
#include "util/maths.h"
#include "util/quaternions.h"
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

AHRS_madgwick::AHRS_madgwick(const Imu& imu, const Airspeed_analog& airspeed, const conf_t& config):
    imu_(imu),
    airspeed_(airspeed),
    attitude_(quat_t{1.0f, {0.0f, 0.0f, 0.0f}}),
    angular_speed_(std::array<float,3>{{0.0f, 0.0f, 0.0f}}),
    linear_acc_(std::array<float,3>{{0.0f, 0.0f, 0.0f}}),
    last_update_us_(0)
{
    // Init config
    beta_                       = config.beta;
    zeta_                       = config.zeta;
    acceleration_correction_    = config.acceleration_correction;
    correction_speed_           = config.correction_speed;

    // Init global variables
    q0_ = 1.0f;
    q1_ = 0.0f;
    q2_ = 0.0f;
    q3_ = 0.0f;
    w_bx_ = 0.0f;
    w_by_ = 0.0f;
    w_bz_ = 0.0f;
}


bool AHRS_madgwick::update(void)
{
    // Compute time
    time_us_t now_us = time_keeper_get_us();
    time_s_t dt_s = (now_us - last_update_us_) / 1e6f;

    std::array<float, 3> acc  = imu_.acc();
    std::array<float, 3> gyro = imu_.gyro();
    std::array<float, 3> mag  = imu_.mag();

    ////////////////////////////////////////////////////
    // Compute correction for parasitic accelerations //
    ////////////////////////////////////////////////////
    // Based on Adrien Briod report about EKF for Quaternion-based orientation estimation, using speed and inertial sensors
    // Centrifugal force
    float hc_y =  - (airspeed_.get_airspeed() * gyro[Z]);
    float hc_z = airspeed_.get_airspeed() * gyro[Y];

    // Longitudinal accelerations
    float hdv_x = - (airspeed_.get_airspeed() - airspeed_.get_last_airspeed())/dt_s;


    /////////////////////////////////////////////////////////////
    // Get measurements with correction, in the MADGWICK FRAME //
    /////////////////////////////////////////////////////////////
    // Transform sensor measurements
    float gx =   gyro[X];
    float gy = - gyro[Y];
    float gz = - gyro[Z];
    float ax =   acc[X];
    float ay = - acc[Y];
    float az = - acc[Z];
    if(acceleration_correction_ == 1 && airspeed_.get_airspeed() >= correction_speed_)
    {
        ax =   (acc[X] + hdv_x);
        ay = - (acc[Y] + hc_y);
        az = - (acc[Z] + hc_z);
    }
    float mx =   mag[X];
    float my = - mag[Y];
    float mz = - mag[Z];


    //////////////////////////////////////////////
    // Run the algorithm, in the MADGWICK FRAME //
    //////////////////////////////////////////////
    madgwick_algo(  gx, gy, gz,
                    ax, ay, az,
                    mx, my, mz,
                    beta_, zeta_, dt_s);


    /////////////////////////////////////////////////////////////////
    // Compute values in MAVRIC FRAME and write back into IMU/AHRS //
    /////////////////////////////////////////////////////////////////
    // Quaternion rotation to express the result in the MAVRIC FRAME
    quat_t q_nwu;
    q_nwu.s = q0_;
    q_nwu.v[X] = q1_;
    q_nwu.v[Y] = q2_;
    q_nwu.v[Z] = q3_;                // Current quaternion in madgwick frame

    quat_t q_nwu2ned;
    q_nwu2ned.s = 0;
    q_nwu2ned.v[X] = 1.0f;
    q_nwu2ned.v[Y] =  0.0f;
    q_nwu2ned.v[Z] =  0.0f;       // Quaternion used for transformation

    quat_t qinv, qtmp;                                      // Transformation q_rot^(-1) * q * q_rot
    qinv = quaternions_inverse(q_nwu2ned);
    qtmp = quaternions_multiply(qinv,q_nwu);
    quat_t q_ned = quaternions_multiply(qtmp,q_nwu2ned);

    // Time
    last_update_us_    = now_us;

    // Quaternion in NED
    attitude_ = q_ned;

    // Angular_speed, subtract estimated biases, making the correspondance between frames !
    angular_speed_[X] = imu_.gyro()[X] - (w_bx_);
    angular_speed_[Y] = imu_.gyro()[Y] - (-w_by_);
    angular_speed_[Z] = imu_.gyro()[Z] - (-w_bz_);

    // Up vector
    float up_loc[3] = {0.0f, 0.0f, -1.0f};
    quat_t up_vec;
    quaternions_rotate_vector(quaternions_inverse(attitude_), up_loc, up_vec.v);

    // Update linear acceleration
    linear_acc_[X] = 9.81f * (imu_.acc()[X] - up_vec.v[X]) ; // TODO: review this line!
    linear_acc_[Y] = 9.81f * (imu_.acc()[Y] - up_vec.v[Y]) ; // TODO: review this line!
    linear_acc_[Z] = 9.81f * (imu_.acc()[Z] - up_vec.v[Z]) ; // TODO: review this line!

    return true;
}


time_us_t AHRS_madgwick::last_update_us(void) const
{
    return last_update_us_;
}


bool AHRS_madgwick::is_healthy(void) const
{
    return true;
}


quat_t AHRS_madgwick::attitude(void) const
{
    return attitude_;
}


std::array<float,3> AHRS_madgwick::angular_speed(void) const
{
    return angular_speed_;
}


std::array<float,3> AHRS_madgwick::linear_acceleration(void) const
{
    return linear_acc_;
}

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void AHRS_madgwick::madgwick_algo(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float beta, float zeta, float dt)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float _w_err_x, _w_err_y, _w_err_z;

    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
    {
        //madgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az, dt);        // TODO: Implement this function using link in brief to run algo even without magnetometer
        return;
    }

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = maths_fast_inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = maths_fast_inv_sqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * q0_ * mx;
        _2q0my = 2.0f * q0_ * my;
        _2q0mz = 2.0f * q0_ * mz;
        _2q1mx = 2.0f * q1_ * mx;
        _2q0 = 2.0f * q0_;
        _2q1 = 2.0f * q1_;
        _2q2 = 2.0f * q2_;
        _2q3 = 2.0f * q3_;
        _2q0q2 = 2.0f * q0_ * q2_;
        _2q2q3 = 2.0f * q2_ * q3_;
        q0q0 = q0_ * q0_;
        q0q1 = q0_ * q1_;
        q0q2 = q0_ * q2_;
        q0q3 = q0_ * q3_;
        q1q1 = q1_ * q1_;
        q1q2 = q1_ * q2_;
        q1q3 = q1_ * q3_;
        q2q2 = q2_ * q2_;
        q2q3 = q2_ * q3_;
        q3q3 = q3_ * q3_;

        // Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * q3_ + _2q0mz * q2_ + mx * q1q1 + _2q1 * my * q2_ + _2q1 * mz * q3_ - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3_ + my * q0q0 - _2q0mz * q1_ + _2q1mx * q2_ - my * q1q1 + my * q2q2 + _2q2 * mz * q3_ - my * q3q3;
        _2bx = maths_fast_sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2_ + _2q0my * q1_ + mz * q0q0 + _2q1mx * q3_ - mz * q1q1 + _2q2 * my * q3_ - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2_ * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3_ + _2bz * q1_) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2_ * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1_ * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3_ * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2_ + _2bz * q0_) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3_ - _4bz * q1_) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2_ * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2_ - _2bz * q0_) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1_ + _2bz * q3_) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0_ - _4bz * q2_) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3_ + _2bz * q1_) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0_ + _2bz * q2_) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1_ * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        recipNorm = maths_fast_inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // compute gyro drift bias
        _w_err_x = _2q0 * s1 - _2q1 * s0 - _2q2 * s3 + _2q3 * s2;
        _w_err_y = _2q0 * s2 + _2q1 * s3 - _2q2 * s0 - _2q3 * s1;
        _w_err_z = _2q0 * s3 - _2q1 * s2 + _2q2 * s1 - _2q3 * s0;

        w_bx_ += _w_err_x * dt * zeta;
        w_by_ += _w_err_y * dt * zeta;
        w_bz_ += _w_err_z * dt * zeta;

        gx -= w_bx_;
        gy -= w_by_;
        gz -= w_bz_;

        // Rate of change of quaternion from gyroscope
        qDot1 = 0.5f * (-q1_ * gx - q2_ * gy - q3_ * gz);
        qDot2 = 0.5f * ( q0_ * gx + q2_ * gz - q3_ * gy);
        qDot3 = 0.5f * ( q0_ * gy - q1_ * gz + q3_ * gx);
        qDot4 = 0.5f * ( q0_ * gz + q1_ * gy - q2_ * gx);

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }
    else
    {
        // Rate of change of quaternion from gyroscope
        qDot1 = 0.5f * (-q1_ * gx - q2_ * gy - q3_ * gz);
        qDot2 = 0.5f * ( q0_ * gx + q2_ * gz - q3_ * gy);
        qDot3 = 0.5f * ( q0_ * gy - q1_ * gz + q3_ * gx);
        qDot4 = 0.5f * ( q0_ * gz + q1_ * gy - q2_ * gx);
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0_ += qDot1 * dt;
    q1_ += qDot2 * dt;
    q2_ += qDot3 * dt;
    q3_ += qDot4 * dt;

    // Normalise quaternion
    recipNorm = maths_fast_inv_sqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
    q0_ *= recipNorm;
    q1_ *= recipNorm;
    q2_ *= recipNorm;
    q3_ *= recipNorm;
}
