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
#include "communication/mavlink_message_handler.hpp"


/**
 * \brief The AHRS EKF class
 */
class Ahrs_ekf_mocap
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
     * \brief   AHRS EKF MOCAP telemetry
     *
     * \param   message_handler The Mavlink message handler
     * \param   ahrs_ekf        The reference to the ahrs_ekf object
     * \param   config          Configuration structure for the mocap
     */
    Ahrs_ekf_mocap(Mavlink_message_handler& message_handler, Ahrs_ekf& ahrs_ekf, const conf_t config_ = Ahrs_ekf_mocap::default_config());

    /**
     * \brief   Initializes the mocap telemetry message and callback
     *
     * \return  Success
     */
    bool init();

    /**
     * \brief   Default configuration structure
     */
    static inline conf_t default_config();


protected:

    /**
     * \brief   Method used to update internal state when a message is received
     *
     * \param   ahrs_ekf_mocap  The ahrs_ekf_mocap object
     * \param   sysid           ID of the system
     * \param   msg             Pointer to the incoming message
     */
    static void callback(Ahrs_ekf_mocap* ahrs_ekf_mocap, uint32_t sysid, mavlink_message_t* msg);

    Ahrs_ekf& ahrs_ekf_;                        ///< AHRS extended callman filter
    Mavlink_message_handler& message_handler_;  ///< State covariance
    Mat<4,4> R_mocap_;                          ///< The mocap measurement noise matrix
    Mat<4, 7> H_;                               ///< The measurement matrix
    Mat<4, 1> z_;                                ///< The measurement vector
    bool is_init_;                              ///< Boolean flag stating if this module has been initialized

    conf_t config_;                                     ///< The config structure for the EKF mocap module

    float last_update_us_;                              ///< The last time the mocap was updated
};

Ahrs_ekf_mocap::conf_t Ahrs_ekf_mocap::default_config()
{
    Ahrs_ekf_mocap::conf_t conf = {};

    conf.R_mocap = 0.000001f;

    return conf;
};

#endif // __AHRS_EKF_MOCAP_HPP__
