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

#include "sensing/ins.hpp"
#include "util/coord_conventions.hpp"
#include "util/constants.hpp"

extern "C"
{
#include <cstdbool>
#include "sensing/ahrs.hpp"
}

// leaky velocity integration as a simple trick to emulate drag and avoid too large deviations (loss per 1 second)
#define VEL_DECAY 0.0f
#define POS_DECAY 0.0f



class Position_estimation: public INS
{
public:

    friend class Mavlink_waypoint_handler;

    /**
     * \brief The position estimator structure
     */
    struct conf_t
    {
        global_position_t origin;   ///<    Global coordinates of the local frame's origin (ie. local (0, 0, 0) expressed in the global frame)
        float gravity;              ///<    value of the Gravity for position estimation correction

        float kp_pos_gps[3];                    ///< Gain to correct the position estimation from the GPS
        float kp_vel_gps[3];                    ///< Gain to correct the velocity estimation from the GPS

        float kp_alt_sonar;                     ///< Gain to correct the Z position estimation from the sonar
        float kp_vel_sonar;                     ///< Gain to correct the Z velocity estimation from the sonar

        float kp_alt_baro;                      ///< Gain to correct the Z position estimation from the barometer
        float kp_vel_baro;                      ///< Gain to correct the Z velocity estimation from the barometer
    };

    enum fence_violation_state_t
    {
        IN_FENCE = 0,
        OUTSIDE_FENCE1,
        OUTSIDE_FENCE2
    };


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
    bool update(void);

    /**
     * \brief   Returns fence violation state
     *
     * \details temporary solution to make fence private
     *          uses state->out_of_fence_1 and state->out_of_fence_2
     *
     * \return  fence violation state   IN_FENCE if inside of fence,
                                        OUTSIDE_FENCE1 if outside of inner fence
                                        OUTSIDE_FENCE2 if outside outer fence (and inner fence)
     */
    fence_violation_state_t get_fence_violation_state() const;

    static conf_t default_config();

    /**
     * \brief   set the home position and altitude
     *
     * \details change of home is only accepted if the vehicle is not armed
     *          THIS IS EVIL: CHANGES ORIGIN OF LOCAL FRAME!!!
     *
     * \param   new_home_pos    new home position in global reference frame
     *
     * \return  accepted    true if new position is accepted;
     */
    bool set_home_position_global(global_position_t new_home_pos);

    /**
     * \brief   set the home position and altitude to currrent position
     *
     * \details calls set_home_position_global with the current position;
     *          only accepted if vehicle is not armed
     *          THIS IS EVIL: CHANGES ORIGIN OF LOCAL FRAME!!!
     *
     * \return  accepted    true if new position is accepted;
     */
    bool set_home_to_current_position();


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



    float kp_alt_baro;                      ///< Gain to correct the Z position estimation from the barometer
    float kp_vel_baro;                      ///< Gain to correct the Z velocity estimation from the barometer
    float kp_pos_gps[3];                    ///< Gain to correct the position estimation from the GPS
    float kp_vel_gps[3];                    ///< Gain to correct the velocity estimation from the GPS
    float kp_alt_sonar;                     ///< Gain to correct the Z position estimation from the sonar
    float kp_vel_sonar;                     ///< Gain to correct the Z velocity estimation from the sonar

private:
    local_position_t local_position;        ///< Local position
    global_position_t origin_;              ///< Local position
    std::array<float,3> vel;                ///< 3D velocity in ned frame
    std::array<float,3> vel_bf;             ///< 3D velocity in body frame

    uint32_t time_last_gps_posllh_msg;      ///< Time at which we received the last GPS POSLLH message in ms
    uint32_t time_last_gps_velned_msg;      ///< Time at which we received the last GPS VELNED message in ms
    uint32_t time_last_barometer_msg;       ///< Time at which we received the last barometer message in ms
    float dt_s_;                            ///< Time interval between updates
    float last_update_s_;                   ///< Last update time in seconds
    bool init_gps_position;                 ///< Boolean flag ensuring that the GPS was initialized
    bool init_barometer;                    ///< Boolean flag ensuring that the barometer was initialized


    float last_alt;                         ///< Value of the last altitude estimation
    float last_vel[3];                      ///< Last 3D velocity

    local_position_t last_gps_pos;          ///< Coordinates of the last GPS position

    local_position_t fence_position;        ///< Position of the fence

    float gravity;                          ///< Value of the gravity

    const ahrs_t& ahrs;                     ///< Reference to the attitude estimation structure
    State& state;                           ///< Reference to the state structure
    const Gps& gps;                         ///< Reference to the GPS structure
    Barometer& barometer;                   ///< Reference to the barometer structure
    const Sonar& sonar;                     ///< Reference to the sonar structure

    /**
     * \brief   Reset the home position and altitude
     *
     */
    void reset_home_position();

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
