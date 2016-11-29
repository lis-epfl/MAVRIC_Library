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
 * \file ahrs_madgwick.hpp
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


/**
 *   Disclaimer: this WIP
 */


#ifndef AHRS_MADGWICK_HPP_
#define AHRS_MADGWICK_HPP_

#include "sensing/ahrs.hpp"
#include "sensing/imu.hpp"
#include "drivers/airspeed_analog.hpp"


class AHRS_madgwick: public AHRS
{
public:
    /**
    * \brief   Configuration for ahrs _madgwick
    */
    struct conf_t
    {
        float   beta;                       // 2 * proportional gain (Kp)
        float   zeta;                       // Gyro drift bias gain
        bool    acceleration_correction;    // Enable the correction of the parasitic accelerations ?
        float   correction_speed;           // Airspeed from which the correction should start
    };


    /**
    * \brief   Default configuration
    */
    static inline conf_t default_config()
    {
        conf_t conf = {};

        conf.beta = 0.06f;
        conf.zeta = 0.0f;
        conf.acceleration_correction = false;
        conf.correction_speed        = 0.0f;

        return conf;
    };


    /**
    * \brief   Init function
    *
    * \param   imu             Reference to IMU structure
    * \param   airspeed        Reference to airspeed sensor
    * \param   config          Config structure
    *
    * \return  True if success, false if not
    */
    AHRS_madgwick(const Imu& imu, const Airspeed_analog& airspeed, const conf_t& config = default_config());


    /**
    * \brief   Main update function
    *
    * \return  success
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
    const Imu&             imu_;                      ///< Reference to IMU sensors
    const Airspeed_analog& airspeed_;                 ///< Reference to the airspeed sensor

    quat_t              attitude_;              ///< Estimated attitude
    std::array<float,3> angular_speed_;         ///< Estimated angular speed
    std::array<float,3> linear_acc_;            ///< Estimated linear acceleration
    float               last_update_s_;         ///< Last update time

    float            beta_;                     ///< 2 * proportional gain (Kp)
    float            zeta_;                     ///< Gyro drift bias gain
    uint32_t         acceleration_correction_;  ///< Enable the correction of the parasitic accelerations ?
    float            correction_speed_;         ///< Airspeed from which the correction should start

private:
    // Global variables used in the Madgwick algorithm.
    // Quaternions used for the Madgwick algorithm. They are in the MADGWICK FRAME, not in the MAVRIC FRAME !
    float q0_, q1_, q2_, q3_;

    // Estimated gyro biases. Warning: they are also expressed in MADGWICK FRAME ! Transformation has to be made to express them in MAVRIC FRAME !
    float w_bx_, w_by_, w_bz_;

    /**
     * \brief   Perform Madgwick algorithm. All parameters (gyro, accelero, magneto, quaternions, biases are expresses in MADGWICK FRAME).
     *
     * \param   gx          Gyro x
     * \param   gx          Gyro y
     * \param   gx          Gyro z
     * \param   gx          Accelero x
     * \param   gx          Accelero y
     * \param   gx          Accelero z
     * \param   gx          Magneto x
     * \param   gx          Magneto y
     * \param   gx          Magneto z
     * \param   beta        Beta gain
     * \param   zeta        Zeta gain
     * \param   dt          Time increment for integration
     */
    void madgwick_algo(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float beta, float zeta, float dt);
};

#endif /* AHRS_MADGWICK_HPP_ */
