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
 * \file qfilter.h
 *
 * \author MAV'RIC Team
 * \author Felix Schill
 *
 * \brief This file implements a complementary filter for the attitude estimation
 *
 ******************************************************************************/


#ifndef QFILTER_HPP_
#define QFILTER_HPP_

#include <cstdint>

#include "sensing/ahrs.hpp"
#include "sensing/imu.hpp"


/**
 * \brief The structure for the quaternion-based attitude estimation
 */
class AHRS_qfilter: public AHRS
{
public:
    /**
     * \brief The structure for configuring the quaternion-based attitude estimation
     */
    struct conf_t
    {
        float   kp;                             ///< The proportional gain for the acceleration correction of the angular rates
        float   ki;                             ///< The integral gain for the acceleration correction of the biais
        float   kp_mag;                         ///< The proportional gain for the magnetometer correction of the angular rates
        float   ki_mag;                         ///< The integral gain for the magnetometer correction of the angular rates
    };


    static inline conf_t default_config()
    {
        conf_t conf = {};

        conf.kp             = 0.07f;
        conf.ki             = 0.0f;
        conf.kp_mag         = 0.1f;
        conf.ki_mag         = 0.0f;

        return conf;
    };


    /**
     * \brief   Constructor
     *
     * \param   imu             The pointer to the IMU structure
     * \param   config          The qfilter configuration gains
     */
    AHRS_qfilter(const Imu& imu, const conf_t& config = default_config());


    /**
     * \brief   Performs the attitude estimation via a complementary filter
     */
    bool update(void);


    /**
     * \brief     Last update in seconds
     *
     * \return    time
     */
    float last_update_s(void) const;


    /**
    * \brief   Indicates which estimate can be trusted
    *
    * \param   type    Type of estimate
    *
    * \return  boolean
    */
    bool is_healthy(void) const;


    /**
     * \brief     Estimated attitude
     *
     * \return    quaternion
     */
    quat_t attitude(void) const;


    /**
     * \brief     Estimated angular velocity
     *
     * \return    3D angular velocity
     */
    std::array<float,3> angular_speed(void) const;


    /**
     * \brief     Estimated linear acceleration
     *
     * \return    3D linear acceleration
     */
    std::array<float,3> linear_acceleration(void) const;


protected:
    const Imu&  imu_;        ///< Pointer to inertial sensors readout

    quat_t              attitude_;              ///< Estimated attitude
    std::array<float,3> angular_speed_;         ///< Estimated angular speed
    std::array<float,3> linear_acc_;            ///< Estimated linear acceleration
    float               last_update_s_;         ///< Last update time

private:
    ahrs_state_t internal_state_;

    float   kp_;             ///< The proportional gain for the acceleration correction of the angular rates
    float   ki_;             ///< The integral gain for the acceleration correction of the biais
    float   kp_mag_;         ///< The proportional gain for the magnetometer correction of the angular rates
    float   ki_mag_;         ///< The integral gain for the magnetometer correction of the angular rates

    float time_s_;           ///< The time keeper to swtich between the internal states

    std::array<float, 3> gyro_bias_;    ///< Gyro bias compensation
};

#endif /* QFILTER_HPP_ */
