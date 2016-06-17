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
 * \file ahrs_ekf_mocap.hpp
 *
 * \author MAV'RIC Team
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
 *
 * Takes into account the motion capture ahrs quaternion
 *
 ******************************************************************************/

#ifndef __AHRS_EKF_MOCAP_HPP__
#define __AHRS_EKF_MOCAP_HPP__

#include "sensing/ahrs_ekf.hpp"
#include "util/matrix.hpp"
#include "util/kalman.hpp"

extern "C"
{
}


/**
 * \brief The AHRS EKF class
 */
class Ahrs_ekf_mocap : public Ahrs_ekf
{
public:

    /**
     * \brief The AHRS EKF MOCAP config structure
     */
    struct conf_t
    {
        float R_mocap;                                  ///< The variance of the accelerometer
    };

    /**
     * \brief   AHRS EKF controller
     *
     * \param   imu             IMU structure (input)
     * \param   ahrs            Attitude estimation structure (output)
     * \param   message_handler The Mavlink message handler
     * \param   config          Configuration structure
     * \param   config_mocap    Configuration structure for the mocap
     */
    Ahrs_ekf_mocap(const Imu& imu, ahrs_t& ahrs, Mavlink_message_handler& message_handler, const Ahrs_ekf::conf_t config = Ahrs_ekf::default_config(), const Ahrs_ekf_mocap::conf_t config_mocap = Ahrs_ekf_mocap::default_config());

    /**
     * \brief   Default configuration structure
     */
    static inline Ahrs_ekf_mocap::conf_t default_config();


protected:

    /**
     * \brief   Method used to update internal state when a message is received
     *
     * \param   ahrs_ekf_mocap  The ahrs_ekf_mocap object
     * \param   sysid           ID of the system
     * \param   msg             Pointer to the incoming message
     */
    static void callback(Ahrs_ekf_mocap* ahrs_ekf_mocap, uint32_t sysid, mavlink_message_t* msg);


    Mat<4,4> R_mocap_;                                  ///< The mocap measurement noise matrix

    conf_t config_mocap_;                               ///< The config structure for the EKF mocap module
};

Ahrs_ekf_mocap::conf_t Ahrs_ekf_mocap::default_config()
{
    Ahrs_ekf_mocap::conf_t conf = {};

    conf.R_mocap = 0.000001f;

    return conf;
};

#endif // __AHRS_EKF_MOCAP_HPP__
