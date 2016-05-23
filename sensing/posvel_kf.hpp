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
 * \file position_kf.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief   Kalman filter for position estimation
 *
 ******************************************************************************/


#ifndef POSVEL_KF_HPP_
#define POSVEL_KF_HPP_


#include "drivers/gps.hpp"
#include "drivers/barometer.hpp"
#include "drivers/sonar.hpp"
#include "sensing/ahrs.h"
#include "sensing/posvel.hpp"

#include "util/kalman.hpp"

extern "C"
{
#include "sensing/altitude.h"
#include "sensing/ahrs.h"
}

/**
 * \brief   Altitude estimator
 *
 * \details
 * - state vector X (n=8): X = [ x_ned, y_ned, z_ned,
                                 z_ground_ned,
                                 vx, vy, vz,
                                 bias_baro ]

   - model A = [ 1, 0, 0, 0, dt, 0,  0,  0 ]  // pos integrated with vel
               [ 0, 1, 0, 0, 0,  dt, 0,  0 ]  // pos integrated with vel
               [ 0, 0, 1, 0, 0,  0,  dt, 0 ]  // pos integrated with vel
               [ 0, 0, 0, 1, 0,  0,  0,  0 ]  // cst ground alt
               [ 0, 0, 0, 0, 1,  0,  0,  0 ]  // cst vel
               [ 0, 0, 0, 0, 0,  1,  0,  0 ]  // cst vel
               [ 0, 0, 0, 0, 0,  0,  1,  0 ]  // cst vel
               [ 0, 0, 0, 0, 0,  0,  0,  1 ]  // cst baro bias

   - input u (n=3):         u = [ ax, ay, az ]

   - input model B = [ (dt^2)/2, 0,        0,        ]
                     [ 0,        (dt^2)/2, 0,        ]
                     [ 0,        0,        (dt^2)/2, ]
                     [ 0,        0,        0,        ]
                     [dt,        0,        0,        ]
                     [ 0,       dt,        0,        ]
                     [ 0,        0,       dt,        ]
                     [ 0,        0,        0         ]

   - measurement 1 (gps) : z1 = [ x_ned, y_ned, z_ned ]
                           H1 = [ 1, 0, 0, 0, 0, 0, 0, 0 ]
                                [ 0, 1, 0, 0, 0, 0, 0, 0 ]
                                [ 0, 0, 1, 0, 0, 0, 0, 0 ]

   - measurement 2 (gps) : z2 = [ vx, vy, vz ]
                           H2 = [ 0, 0, 0, 0, 1, 0, 0, 0 ]
                                [ 0, 0, 0, 0, 0, 1, 0, 0 ]
                                [ 0, 0, 0, 0, 0, 0, 1, 0 ]

   - measurement 3 (baro): z3 = [ z_ned + bias_baro ]
                           H3 = [ 0, 0, 1, 0, 0, 0, 0, 1 ]

   - measurement 4 (sonar): z4 = [ z_ned - z_ground_ned ]
                            H4 = [ 0, 0, 1, -1, 0, 0, 0, 0 ]


 */
class Posvel_kf: public Kalman<8,3,3>
{
public:
    /**
     * \brief   Configuration structure
     */
    struct conf_t
    {
        float dt;
        float sigma_gps_xy;
        float sigma_gps_z;
        global_position_t home;
    };

    /**
     * \brief Constructor
     */
    Posvel_kf(const Gps& gps,
              const Barometer& barometer,
              const Sonar& sonar,
              const ahrs_t& ahrs,
              posvel_t& posvel,
              const conf_t config = default_config() );

    /**
     * \brief   Initialization
     *
     * \return  Success
     */
    bool init(void);

    /**
     * \brief   Main update function
     *
     * \return  Success
     */
    bool update(void);

    /**
     * \brief   Default configuration structure
     */
    static inline Posvel_kf::conf_t default_config(void);


private:
    const Gps&          gps_;             ///< Gps (input)
    const Barometer&    barometer_;       ///< Barometer (input)
    const Sonar&        sonar_;           ///< Sonar, must be downward facing (input)
    const ahrs_t&       ahrs_;            ///< Attitude and acceleration (input)
    posvel_t&           posvel_;        ///< Estimated altitude (output)

    conf_t config_;                      ///< Configuration

    Mat<3,8> H_gpsvel_;
    Mat<3,3> R_gpsvel_;
    Mat<1,8> H_baro_;
    Mat<1,1> R_baro_;
    Mat<1,8> H_sonar_;
    Mat<1,1> R_sonar_;

    float last_accel_update_s_;          ///< Last time we updated the estimate using accelerometer
    float last_sonar_update_s_;          ///< Last time we updated the estimate using sonar
    float last_baro_update_s_;           ///< Last time we updated the estimate using barometer
    float last_gps_pos_update_s_;        ///< Last time we updated the estimate using gps position
    float last_gps_vel_update_s_;        ///< Last time we updated the estimate using gps velocity
};


Posvel_kf::conf_t Posvel_kf::default_config(void)
{
    Posvel_kf::conf_t conf = {};

    conf.dt = 0.004f;

    conf.sigma_gps_xy = 2.0f;
    conf.sigma_gps_z  = 3.0f;

    //default home location (EFPL Esplanade)
    conf.home.longitude           = 6.566044801857777f;
    conf.home.latitude            = 46.51852236174565f;
    conf.home.altitude            = 400.0f;
    conf.home.heading             = 0.0f;

    return conf;
};

static inline bool task_posvel_kf_update(Posvel_kf* posvel_kf)
{
    return posvel_kf->update();
}

#endif /* POSVEL_KF_HPP_ */
