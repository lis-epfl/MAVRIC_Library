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
 * \file ahrs_ekf.hpp
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

#ifndef __AHRS_EKF_HPP__
#define __AHRS_EKF_HPP__

#include "sensing/ahrs.hpp"
#include "sensing/imu.hpp"
#include "util/kalman.hpp"
#include "util/matrix.hpp"


/**
 * \brief The AHRS EKF class
 */
class Ahrs_ekf : public Kalman<7, 0, 3>
{
public:

    /**
     * \brief The AHRS EKF config structure
     */
    struct conf_t
    {
        float sigma_w_sqr;                            ///< The square of the variance on the gyro bias
        float sigma_r_sqr;                            ///< The square of the variance on the quaternion

        float acc_norm_noise;                         ///< The noise gain depending on the norm of the acceleration
        float acc_multi_noise;                        ///< The multiplication factor in the computation of the noise for the accelerometer

        float R_acc;                                  ///< The variance of the accelerometer
        float R_mag;                                  ///< The variance of the magnetometer
    };

    /**
     * \brief   AHRS EKF controller
     *
     * \param   imu             IMU structure (input)
     * \param   ahrs            Attitude estimation structure (output)
     * \param   config          Configuration structure
     */
    Ahrs_ekf(const Imu& imu, ahrs_t& ahrs, const Ahrs_ekf::conf_t config = default_config());

    /**
     * \brief   Performs the EKF algorithm
     *
     * \return  true if success
     */
    bool update(void);

    /**
     * \brief   Performs the north vector calibration
     */
    void calibrating_north_vector(void);

    /**
     * \brief   Default configuration structure
     */
    static inline Ahrs_ekf::conf_t default_config();

protected:

    /**
     * \brief   Initialize the state and matrix of the EKF
     */
    void init_kalman(void);

    /**
     * \brief   Performs the prediction step of the EKF
     */
    void predict_step(void);

    /**
     * \brief   Performs the update step with the accelerometer
     */
    void update_step_acc(void);

    /**
     * \brief   Performs the update step with the magnetometer
     */
    void update_step_mag(void);


    Mat<3,3> R_acc_;                                    ///< The accelerometer measruement noise matrix
    Mat<3,3> R_mag_;                                    ///< The magnetometer measurement noise matrix

    float dt_s_;
    float last_update_s_;

    conf_t config_;                                     ///< The config structure for the EKF module

    const Imu& imu_;                                    ///< The Reference to the IMU structure
    ahrs_t& ahrs_;                                      ///< The pointer to the ahrs structure
};

Ahrs_ekf::conf_t Ahrs_ekf::default_config()
{
    Ahrs_ekf::conf_t conf = {};

    conf.sigma_w_sqr = 0.0000000001f;
    conf.sigma_r_sqr = 0.000001f;
    conf.R_acc = 0.004f;
    conf.R_mag = 0.040f;
    conf.acc_norm_noise = 0.05f;
    conf.acc_multi_noise = 4.0f;

    return conf;
};

#endif // __AHRS_EKF_HPP__
