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
 * \file ins_complementary.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief This file performs the 3D position estimation, either by direct
 * integration or with correction with the GPS and pos_est->barometer
 *
 ******************************************************************************/


#ifndef INS_COMPLEMENTARY_HPP__
#define INS_COMPLEMENTARY_HPP__

#include <cstdbool>

#include "status/state.hpp"
#include "drivers/gps.hpp"
#include "drivers/barometer.hpp"
#include "drivers/sonar.hpp"
#include "drivers/px4flow_i2c.hpp"

#include "sensing/ahrs.hpp"
#include "sensing/ins.hpp"
#include "util/coord_conventions.hpp"
#include "util/constants.hpp"

class INS_complementary: public INS
{
public:

    friend class Mavlink_waypoint_handler;

    /**
     * \brief   Configuration structure
     */
    struct conf_t
    {
        global_position_t origin;   ///<    Global coordinates of the local frame's origin (ie. local (0, 0, 0) expressed in the global frame)

        float kp_gps_XY_pos;                    ///< Gain to correct the XY position estimation from the GPS
        float kp_gps_Z_pos;                     ///< Gain to correct the Z position estimation from the GPS
        float kp_gps_XY_vel;                    ///< Gain to correct the XY velocity estimation from the GPS
        float kp_gps_Z_vel;                     ///< Gain to correct the Z velocity estimation from the GPS
        float kp_gps_XY_pos_dgps;               ///< Gain to correct the XY position estimation from the GPS when it has DPGS fix
        float kp_gps_Z_pos_dgps;                ///< Gain to correct the Z position estimation from the GPS when it has DPGS fix
        float kp_gps_XY_vel_dgps;               ///< Gain to correct the XY velocity estimation from the GPS when it has DPGS fix
        float kp_gps_Z_vel_dgps;                ///< Gain to correct the Z velocity estimation from the GPS when it has DPGS fix
        float kp_gps_XY_pos_rtk;                ///< Gain to correct the XY position estimation from the GPS when it has RTK fix
        float kp_gps_Z_pos_rtk;                 ///< Gain to correct the Z position estimation from the GPS when it has RTK fix
        float kp_gps_XY_vel_rtk;                ///< Gain to correct the XY velocity estimation from the GPS when it has RTK fix
        float kp_gps_Z_vel_rtk;                 ///< Gain to correct the Z velocity estimation from the GPS when it has RTK fix
        float timeout_gps_us;                   ///< Time after witch a measure stops being used
        uint32_t use_gps;                       ///< Boolean that indicates if the sensor must be used

        float kp_sonar_alt;                     ///< Gain to correct the Z position estimation from the sonar
        float kp_sonar_vel;                     ///< Gain to correct the Z velocity estimation from the sonar
        float timeout_sonar_us;                 ///< Time after witch a measure stops being used
        uint32_t use_sonar;                     ///< Boolean that indicates if the sensor must be used

        float kp_baro_alt;                      ///< Gain to correct the Z position estimation from the barometer
        float kp_baro_vel;                      ///< Gain to correct the Z velocity estimation from the barometer
        float timeout_baro_us;                  ///< Time after witch a measure stops being used
        uint32_t use_baro;                      ///< Boolean that indicates if the sensor must be used

        float kp_flow_vel;                      ///< Gain to correct the XY velocity estimation from optical flow
        float timeout_flow_us;                  ///< Time after witch a measure stops being used
        uint32_t use_flow;                      ///< Boolean that indicates if the sensor must be used
    };


    enum fence_violation_state_t
    {
        IN_FENCE = 0,
        OUTSIDE_FENCE1,
        OUTSIDE_FENCE2
    };


    /**
     * \brief   Initialize the module
     *
     * \param   state           state structure
     * \param   barometer       barometer structure
     * \param   sonar           sonar structure
     * \param   gps             GPS structure
     * \param   flow            Optic flow structure
     * \param   ahrs            attitude estimation structure
     * \param   config          configuration
     *
     * \return  True if the init succeed, false otherwise
     */
    INS_complementary(State& state, const Barometer& barometer, const Sonar& sonar, const Gps& gps, const Px4flow_i2c& flow, const ahrs_t& ahrs, const conf_t config = default_config());


    /**
     * \brief   Position estimation update step, performing position estimation then position correction (function to be used)
     *
     * \return Success
     */
    bool update(void);


    static inline conf_t default_config();

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


    local_position_t local_position_;        ///< Local position
    float vel_[3];                           ///< 3D velocity in ned frame
    conf_t config_;                          ///< Configuration containing gains

private:
    const ahrs_t& ahrs_;                     ///< Reference to the attitude estimation structure
    State& state_;                           ///< Reference to the state structure
    const Gps& gps_;                         ///< Reference to the GPS structure
    const Barometer& barometer_;             ///< Reference to the barometer structure
    const Sonar& sonar_;                     ///< Reference to the sonar structure
    const Px4flow_i2c& flow_;                ///< Reference to the flow structure

    float dt_s_;                            ///< Time interval between updates
    float last_update_s_;                   ///< Last update time in seconds
    uint64_t last_gps_pos_update_us_;       ///< Time at which we did correction using GPS position in us
    uint64_t last_gps_vel_update_us_;       ///< Time at which we did correction using GPS velocity in us
    uint64_t last_barometer_update_us_;     ///< Time at which we did correction using barometer in us
    uint64_t last_sonar_update_us_;         ///< Time at which we did correction using sonar in us
    uint64_t last_flow_update_us_;          ///< Time at which we did correction using optic flow in us

    bool is_gps_pos_initialized_;           ///< Boolean flag ensuring that the GPS was initialized
    local_position_t fence_position_;        ///< Position of the fence

    float barometer_bias_;                   ///< Altitude bias between actual altitude and altitude given by the barometer
    bool is_barometer_calibrated_;           ///< Flag that indicates if the barometer was calibrated

    /**
     * \brief   Reset the velocity and altitude estimation to 0 and corrects barometer bias
     *
     */
    void reset_velocity_altitude(void);


    /**
     * \brief   Direct integration of the position with the IMU data
     *
     */
    void integration(void);


    /**
     * \brief   State correction with gps position
     */
    void correction_from_gps_pos(void);


    /**
     * \brief   State correction with gps velocity
     */
    void correction_from_gps_vel(void);


    /**
     * \brief   State correction with gps
     */
    void correction_from_gps(void);


    /**
     * \brief   State correction with barometer
     */
    void correction_from_barometer(void);


    /**
     * \brief   State correction with sonar
     */
    void correction_from_sonar(void);


    /**
     * \brief   State correction with optic flow
     */
    void correction_from_flow(void);


    /**
     * \brief   Initialization of the position estimation from the GPS position
     *
     * \param   gps             The pointer to the GPS structure
     *
     * \return  void
     */
    void check_first_gps_fix(void);


    /**
     * \brief   Calibrate the barometer (update the bias)
     *
     * \return  void
     */
    void calibrate_barometer(void);

};

INS_complementary::conf_t INS_complementary::default_config()
{
    conf_t conf = {};

    // default origin location (EFPL Esplanade)
    conf.origin        = ORIGIN_EPFL;

    // GPS with 3d fix
    conf.kp_gps_XY_pos  = 2.0f;
    conf.kp_gps_Z_pos   = 1.0f;
    conf.kp_gps_XY_vel  = 2.0f;
    conf.kp_gps_Z_vel   = 1.0f;
    conf.timeout_gps_us = 1e6f;
    conf.use_gps        = 1;

    // GPS with DGPS fix
    conf.kp_gps_XY_pos  = 20.0f;
    conf.kp_gps_Z_pos   = 10.0f;
    conf.kp_gps_XY_vel  = 20.0f;
    conf.kp_gps_Z_vel   = 10.0f;

    // GPS with RTK fix
    conf.kp_gps_XY_pos  = 200.0f;
    conf.kp_gps_Z_pos   = 100.0f;
    conf.kp_gps_XY_vel  = 200.0f;
    conf.kp_gps_Z_vel   = 100.0f;


    // Barometer
    conf.kp_baro_alt     = 2.0f;
    conf.kp_baro_vel     = 0.5f;
    conf.timeout_baro_us = 1e6f;
    conf.use_baro        = 1;

    // Sonar
    conf.kp_sonar_alt       = 5.0f;
    conf.kp_sonar_vel       = 3.0f;
    conf.timeout_sonar_us   = 1e6f;
    conf.use_sonar          = 1;

    // Optic flow
    conf.kp_flow_vel     = 4.0f;
    conf.timeout_flow_us = 1e6f;
    conf.use_flow        = 1;

    return conf;
};

#endif // INS_COMPLEMENTARY_HPP__
