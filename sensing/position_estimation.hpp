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
 * \file position_estimation.h
 *
 * \author MAV'RIC Team
 *
 * \brief This file performs the 3D position estimation, either by direct
 * integration or with correction with the GPS and pos_est->barometer
 *
 ******************************************************************************/


#ifndef POSITION_ESTIMATION_H__
#define POSITION_ESTIMATION_H__


#include "communication/state.hpp"
#include "drivers/gps.hpp"
#include "drivers/barometer.hpp"
#include "drivers/sonar.hpp"

extern "C"
{
#include <stdbool.h>
#include "sensing/ahrs.h"
#include "util/coord_conventions.h"
#include "util/constants.h"
}

// leaky velocity integration as a simple trick to emulate drag and avoid too large deviations (loss per 1 second)
#define VEL_DECAY 0.0f
#define POS_DECAY 0.0f



class Position_estimation
{
public:

    friend class Mavlink_waypoint_handler;

    /**
     * \brief The position estimator structure
     */
    typedef struct
    {
        global_position_t origin;   ///<    Global coordinates of the local frame's origin (ie. local (0, 0, 0) expressed in the global frame)
        float gravity;              ///<    value of the Gravity for position estimation correction
        bool fence_set;
    }conf_t;


    /**
     * \brief   Initialize the position estimation module
     *
     * \param   state           state structure
     * \param   barometer       barometer structure
     * \param   sonar           sonar structure
     * \param   gps             GPS structure
     * \param   ahrs            attitude estimation structure
     * \param   config          default home position and gravity value
     *
     * \return  True if the init succeed, false otherwise
     */
    Position_estimation(State& state, Barometer& barometer, const Sonar& sonar, const Gps& gps, const ahrs_t& ahrs, const conf_t config = default_config());

    /**
     * \brief   Position estimation update step, performing position estimation then position correction (function to be used)
     *
     */
    void update();

    
    void set_fence_to_current_position();

    static conf_t default_config();

    local_position_t local_position;        ///< Local position
    float vel[3];                           ///< 3D velocity in global frame
    float kp_alt_baro;                      ///< Gain to correct the Z position estimation from the barometer
    float kp_vel_baro;                      ///< Gain to correct the Z velocity estimation from the barometer
    float kp_pos_gps[3];                    ///< Gain to correct the position estimation from the GPS
    float vel_bf[3];                        ///< 3D velocity in body frame
    const Gps& gps;                         ///< Reference to the GPS structure
    State& state;                           ///< Reference to the state structure
    const ahrs_t& ahrs;                       ///< Reference to the attitude estimation structure
    bool* nav_plan_active;                  ///< Pointer to the waypoint set flag
private:
    float kp_vel_gps[3];                    ///< Gain to correct the velocity estimation from the GPS
    float kp_alt_sonar;                     ///< Gain to correct the Z position estimation from the sonar
    float kp_vel_sonar;                     ///< Gain to correct the Z velocity estimation from the sonar

    uint32_t time_last_gps_posllh_msg;      ///< Time at which we received the last GPS POSLLH message in ms
    uint32_t time_last_gps_velned_msg;      ///< Time at which we received the last GPS VELNED message in ms
    uint32_t time_last_barometer_msg;       ///< Time at which we received the last barometer message in ms
    bool init_gps_position;                 ///< Boolean flag ensuring that the GPS was initialized
    bool init_barometer;                    ///< Boolean flag ensuring that the barometer was initialized


    float last_alt;                         ///< Value of the last altitude estimation
    float last_vel[3];                      ///< Last 3D velocity

    local_position_t last_gps_pos;          ///< Coordinates of the last GPS position

    bool fence_set;                         ///< Indicates if fence is set
    local_position_t fence_position;        ///< Position of the fence

    float gravity;                          ///< Value of the gravity

    Barometer& barometer;                   ///< Reference to the barometer structure
    const Sonar& sonar;                     ///< Reference to the sonar structure


    /**
     * \brief   Reset the home position and altitude
     *
     */
    void reset_home_position();

    /**
     * \brief   Reset the origin of the fence (e.g. common for many entities or when armed)
     *
     */
    void reset_fence_origin();

    /**
     * \brief   Direct integration of the position with the IMU data
     *
     */
    void position_integration();

    /**
     * \brief   Position correction with the GPS and the barometer
     *
     */
    void position_correction();

    /**
     * \brief   Initialization of the position estimation from the GPS position
     *
     * \param   gps             The pointer to the GPS structure
     *
     * \return  void
     */
    void gps_position_init();

    /**
     * \brief   Check if the robot is going further from the working radius, delimited by those fences
     *
     * \return  void
     */
    void fence_control();

};
#endif // POSITION_ESTIMATION_H__