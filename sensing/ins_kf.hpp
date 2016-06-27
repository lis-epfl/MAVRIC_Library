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
 * \file ins_kf.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief   Kalman filter for position estimation
 *
 ******************************************************************************/


#ifndef INS_KF_HPP_
#define INS_KF_HPP_


#include "drivers/gps.hpp"
#include "drivers/barometer.hpp"
#include "drivers/sonar.hpp"
#include "drivers/px4flow_i2c.hpp"
#include "sensing/ahrs.hpp"
#include "sensing/ins.hpp"

#include "util/kalman.hpp"

extern "C"
{
#include "sensing/altitude.h"
#include "sensing/ahrs.hpp"
}

/**
 * \brief   Altitude estimator
 *
 * \details
 * - state vector X (n=8): X = [ x_ned, y_ned, z_ned,
                                 z_ground_abs,
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

   - measurement 1 (gps) : z1 = [ x_ned, y_ned, z_abs ]
                           H1 = [ 1, 0, 0, 0, 0, 0, 0, 0 ]
                                [ 0, 1, 0, 0, 0, 0, 0, 0 ]
                                [ 0, 0, 0, 1, 0, 0, 0, 0 ]

   - measurement 2 (gps) : z2 = [ vx, vy, vz ]
                           H2 = [ 0, 0, 0, 0, 1, 0, 0, 0 ]
                                [ 0, 0, 0, 0, 0, 1, 0, 0 ]
                                [ 0, 0, 0, 0, 0, 0, 1, 0 ]

   - measurement 3 (baro): z3 = [ z_abs + bias_baro ]
                           H3 = [ 0, 0, 0, 1, 0, 0, 0, 1 ]

   - measurement 4 (sonar): z4 = [ -z_ned ]
                            H4 = [ 0, 0, -1, 0, 0, 0, 0, 0 ]
 */
class INS_kf: public Kalman<8,3,3>, public INS
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
    INS_kf(const Gps& gps,
              const Barometer& barometer,
              const Sonar& sonar,
              const Px4flow_i2c& flow,
              const ahrs_t& ahrs,
              const conf_t config = default_config() );


    /**
     * \brief   Main update function
     *
     * \return  Success
     */
    bool update(void);


    /**
     * \brief     Last update in seconds
     *
     * \return    time
     */
    float last_update_s(void) const;


    /**
     * \brief     3D Position in meters (NED frame)
     *
     * \return    position
     */
    std::array<float,3> position_lf(void) const;


    /**
     * \brief     Position of origin in global coordinates
     *
     * \return    origin
     */
    // virtual global_position_t origin(void) const = 0;


    /**
     * \brief     Velocity in meters/seconds in NED frame
     *
     * \return    velocity
     */
    std::array<float,3> velocity_lf(void) const;


    /**
     * \brief     Absolute altitude above sea level in meters (>=0)
     *
     * \return    altitude
     */
    float absolute_altitude(void) const;


    /**
     * \brief   Indicates which estimate can be trusted
     *
     * \param   type    Type of estimate
     *
     * \return  boolean
     */
    bool is_healthy(INS::healthy_t type) const;


    /**
     * \brief   Default configuration structure
     */
    static inline INS_kf::conf_t default_config(void);


private:
    const Gps&          gps_;             ///< Gps (input)
    const Barometer&    barometer_;       ///< Barometer (input)
    const Sonar&        sonar_;           ///< Sonar, must be downward facing (input)
    const Px4flow_i2c&  flow_;            ///< Optical flow sensor (input)
    const ahrs_t&       ahrs_;            ///< Attitude and acceleration (input)

    conf_t config_;                      ///< Configuration

    std::array<float,3> pos_;
    std::array<float,3> vel_;
    float absolute_altitude_;

    Mat<3,8> H_gpsvel_;
    Mat<3,3> R_gpsvel_;
    Mat<1,8> H_baro_;
    Mat<1,1> R_baro_;
    Mat<1,8> H_sonar_;
    Mat<1,1> R_sonar_;
    Mat<3,8> H_flow_;
    Mat<3,3> R_flow_;

    float last_accel_update_s_;          ///< Last time we updated the estimate using accelerometer
    float last_sonar_update_s_;          ///< Last time we updated the estimate using sonar
    float last_flow_update_s_;           ///< Last time we updated the estimate using optical flow
    float last_baro_update_s_;           ///< Last time we updated the estimate using barometer
    float last_gps_pos_update_s_;        ///< Last time we updated the estimate using gps position
    float last_gps_vel_update_s_;        ///< Last time we updated the estimate using gps velocity
};


INS_kf::conf_t INS_kf::default_config(void)
{
    INS_kf::conf_t conf = {};

    conf.dt = 0.004f;

    // conf.sigma_gps_xy = 2.0f;
    // conf.sigma_gps_z  = 3.0f;

    // Low values for optitrack
    conf.sigma_gps_xy = 0.001f;
    conf.sigma_gps_z  = 0.001f;


    //default home location (EFPL Esplanade)
    conf.home.longitude           = 6.566044801857777f;
    conf.home.latitude            = 46.51852236174565f;
    conf.home.altitude            = 400.0f;

    return conf;
};

static inline bool task_ins_kf_update(INS_kf* ins_kf)
{
    return ins_kf->update();
}

#endif /* INS_KF_HPP_ */
