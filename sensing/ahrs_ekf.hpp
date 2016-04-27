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

#include "util/matrix.hpp"
#include "sensing/imu.hpp"

extern "C" 
{
#include "sensing/ahrs.h"
}

/**
 * \brief The AHRS EKF config structure
 */
typedef struct
{
    float sigma_w_sqr;                                  ///< The square of the variance on the gyro bias
    float sigma_r_sqr;                                  ///< The square of the variance on the quaternion

    float acc_norm_noise;                               ///< The noise gain depending on the norm of the acceleration
    float acc_multi_noise;                              ///< The multiplication factor in the computation of the noise for the accelerometer

    float R_acc;                                        ///< The variance of the accelerometer
    float R_mag;                                        ///< The variance of the magnetometer

    float mag_global[3];                                ///< The value of the north vector
}ahrs_ekf_conf_t;


/**
 * \brief The AHRS EKF class
 */
class Ahrs_ekf
{
public:
    /**
     * \brief   AHRS EKF controller
     *
     * \param   config          The reference to the ahrs_ekf configuration structure
     * \param   imu             The reference to the IMU structure
     * \param   ahrs            The pointer to the AHRS structure
     */
    Ahrs_ekf(const ahrs_ekf_conf_t& config, Imu& imu, ahrs_t* ahrs);

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
     * \brief   Returns the flag telling whether the module is in calibration mode or not
     *
     * \return  true if not in calibration mode, false otherwise
     */
    const bool is_ready(void) const;

    float mag_global_[3];                               ///< The magnetic North vector
    float mag_lpf_[3];                                  ///< The magnetometer low pass filter for North vector calibration
    Imu& imu_;                                          ///< The Reference to the IMU structure
    bool calibrating_north_vector_;                     ///< Flag to start the north vector calibration

private:

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

    Mat<7,1> x_state_;                                  ///< The state of extended Kalman filter
    Mat<7,7> F_;                                        ///< The state transition matrix
    Mat<7,7> P_;                                        ///< The state covariance matrix
    Mat<7,7> Q_;                                        ///< The process noise covariance matrix
    Mat<3,3> R_acc_;                                    ///< The accelerometer measruement noise matrix
    Mat<3,3> R_mag_;                                    ///< The magnetometer measurement noise matrix
    Mat<7,7> Id_;                                       ///< The 7x7 identity matrix

    ahrs_t* ahrs_;                                      ///< The pointer to the ahrs structure
    ahrs_ekf_conf_t config_;                            ///< The config structure for the EKF module
};




#endif // __AHRS_EKF_HPP__