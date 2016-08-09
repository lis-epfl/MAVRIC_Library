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
 * \file position_estimation.cpp
 *
 * \author MAV'RIC Team
 *
 * \brief This file performs the 3D position estimation, either by direct
 * integration or with correction with the GPS and pos_est->barometer
 *
 ******************************************************************************/


#include "sensing/position_estimation.hpp"
#include "drivers/barometer.hpp"
#include "hal/common/time_keeper.hpp"

extern "C"
{
#include "util/print_util.h"
#include "util/maths.h"
}

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void Position_estimation::position_integration()
{
    int32_t i;
    float dt = dt_s_;

    quat_t qvel_bf, qvel;

    qvel.s = 0;
    for (i = 0; i < 3; i++)
    {
        qvel.v[i] = vel[i];
    }
    qvel_bf = quaternions_global_to_local(ahrs.qe, qvel);
    for (i = 0; i < 3; i++)
    {
        vel_bf[i] = qvel_bf.v[i];
        vel_bf[i] = vel_bf[i] * (1.0f - (VEL_DECAY * dt)) + ahrs.linear_acc[i]  * dt;
    }

    // calculate velocity in global frame
    // vel = qe *vel_bf * qe - 1

    qvel_bf.s = 0.0f;
    qvel_bf.v[0] = vel_bf[0];  // TODO: replace this by quat_rotate_vector()
    qvel_bf.v[1] = vel_bf[1];
    qvel_bf.v[2] = vel_bf[2];

    qvel = quaternions_local_to_global(ahrs.qe, qvel_bf);

    vel[0] = qvel.v[0];
    vel[1] = qvel.v[1];
    vel[2] = qvel.v[2];    // TODO: replace this by quat_rotate_vector()

    for (i = 0; i < 3; i++)
    {
        local_position[i] = local_position[i] * (1.0f - (POS_DECAY * dt)) + vel[i] * dt;
    }

}


void Position_estimation::position_correction()
{
    global_position_t global_gps_position;
    local_position_t local_coordinates;

    float gps_gain = 0.0f;
    float baro_alt_error = 0.0f;
    float baro_vel_error = 0.0f;
    float baro_gain = 0.0f;
    float sonar_alt_error = 0.0f;
    float sonar_vel_error = 0.0f;
    float sonar_gain = 0.0f;
    float gps_dt = 0.0f;
    float dt = dt_s_;

    // // quat_t bias_correction = {.s = 0, .v = {0.0f, 0.0f, 1.0f}};
    // quat_t vel_correction = {};
    // vel_correction.s       = 0.0f;
    // vel_correction.v[0]   = 0.0f;
    // vel_correction.v[1]   = 0.0f;
    // vel_correction.v[2]   = 0.0f;

    float pos_error[3] =
    {
        0.0f,
        0.0f,
        0.0f
    };


    float vel_error[3] =
    {
        0.0f,
        0.0f,
        0.0f
    };

    // uint32_t t_inter_baro;
    int32_t i;

    if (barometer.has_been_calibrated())
    {
        // altimeter correction
        if (time_last_barometer_msg < barometer.last_update_us())
        {
            last_alt = - (barometer.altitude_gf() - origin_.altitude);

            time_last_barometer_msg = barometer.last_update_us();
        }

        baro_gain = 1.0f;

        baro_alt_error = last_alt  - local_position[Z];
        baro_vel_error = barometer.vertical_speed_lf() - vel[Z];
    }
    else
    {
        // Wait for gps to initialized as we need an absolute altitude
        if (init_gps_position)
        {
            // Correct barometer bias
            float current_altitude_gf = - local_position[Z] + origin_.altitude;
            barometer.calibrate_bias(current_altitude_gf);
        }
    }

    if (init_gps_position)
    {
        if (gps.healthy() == true)
        {
            if ((time_last_gps_posllh_msg < gps.last_position_update_us()))
            {
                global_gps_position = gps.position_gf();
                coord_conventions_global_to_local_position(global_gps_position, origin_, local_coordinates);

                // compute GPS velocity estimate
                gps_dt = (gps.last_position_update_us() - time_last_gps_posllh_msg) / 1000000.0f;
                if (gps_dt > 0.001f)
                {
                    for (i = 0; i < 3; i++)
                    {
                        last_vel[i] = (local_coordinates[i] - last_gps_pos[i]) / gps_dt;
                    }
                }
                else
                {
                    print_util_dbg_print("GPS dt is too small!\r\n");
                }

                time_last_gps_posllh_msg = gps.last_position_update_us();
                last_gps_pos = local_coordinates;
            }

            if (time_last_gps_velned_msg < gps.last_velocity_update_us())
            {
                time_last_gps_velned_msg = gps.last_velocity_update_us();
            }

            // Enable gps correction
            gps_gain = 1.0f;

            // Compute position error and velocity error from gps
            for (i = 0; i < 3; i++)
            {
                pos_error[i] = last_gps_pos[i] - local_position[i];
                vel_error[i] = gps.velocity_lf()[i] - vel[i];
            }
        }
        else
        {
            // Disable gps correction
            gps_gain = 0.0f;

            // Do not use position error and velocity error from GPS
            for (i = 0; i < 3; i++)
            {
                pos_error[i] = 0.0f;
                vel_error[i] = 0.0f;
            }
        }
    }
    else
    {
        gps_position_init();
        for (i = 0; i < 3; i++)
        {
            pos_error[i] = 0.0f;
            vel_error[i] = 0.0f;
        }
        gps_gain = 0.0f;
    }

    if (sonar.healthy())
    {
        sonar_gain      = 1.0f;
        sonar_alt_error = - sonar.distance() - local_position[Z];
        sonar_vel_error = - sonar.velocity() - vel[Z];
    }
    else
    {
        sonar_gain      = 0.0f;
        sonar_alt_error = 0.0f;
        sonar_vel_error = 0.0f;
    }

    // Apply error correction to position estimates
    for (i = 0; i < 3; i++)
    {
        local_position[i] += kp_pos_gps[i] * gps_gain * pos_error[i] * dt;
    }
    local_position[Z] += kp_alt_baro * baro_gain * baro_alt_error * dt;
    local_position[Z] += kp_alt_sonar * sonar_gain * sonar_alt_error * dt;

    // Apply error correction to velocity estimates
    for (i = 0; i < 3; i++)
    {
        vel[i] += kp_vel_gps[i] * gps_gain * vel_error[i] * dt;
    }
    vel[Z] += kp_vel_baro * baro_gain * baro_vel_error * dt;
    vel[Z] += kp_vel_sonar * sonar_gain * sonar_vel_error * dt;
}


void Position_estimation::gps_position_init()
{
    if ((init_gps_position == false) && (gps.healthy() == true))
    {
        if ((time_last_gps_posllh_msg < gps.last_position_update_us())
                && (time_last_gps_velned_msg < gps.last_velocity_update_us()))
        {
            time_last_gps_posllh_msg = gps.last_position_update_us();
            time_last_gps_velned_msg = gps.last_velocity_update_us();

            init_gps_position = true;

            origin_  = gps.position_gf();
            last_gps_pos           = local_position;

            last_alt = 0;
            for (int32_t i = 0; i < 3; i++)
            {
                last_vel[i] = 0.0f;
                local_position[i] = 0.0f;
                vel[i] = 0.0f;
            }

            print_util_dbg_print("GPS position initialized!\r\n");
        }
    }
}

void Position_estimation::fence_control()
{
    float dist_xy_sqr, dist_z_sqr;
    dist_xy_sqr = SQR(local_position[X] - fence_position[X]) + SQR(local_position[Y] - fence_position[Y]);
    dist_z_sqr = SQR(local_position[Z] - fence_position[Z]);

    if (dist_xy_sqr > SQR(state.fence_2_xy))
    {
        state.out_of_fence_2 = true;
    }
    else if (dist_z_sqr > SQR(state.fence_2_z))
    {
        state.out_of_fence_2 = true;
    }
    else if (dist_xy_sqr > SQR(state.fence_1_xy))
    {
        state.out_of_fence_1 = true;
    }
    else if (dist_z_sqr > SQR(state.fence_1_z))
    {
        state.out_of_fence_1 = true;
    }
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Position_estimation::Position_estimation(State& state, Barometer& barometer, const Sonar& sonar, const Gps& gps, const ahrs_t& ahrs, const conf_t config) :
        kp_alt_baro(config.kp_alt_baro),
        kp_vel_baro(config.kp_vel_baro),
        kp_alt_sonar(config.kp_alt_sonar),
        kp_vel_sonar(config.kp_vel_sonar),
        vel(std::array<float,3>{{0.0f,0.0f,0.0f}}),
        vel_bf(std::array<float,3>{{0.0f,0.0f,0.0f}}),
        time_last_gps_posllh_msg(0),
        time_last_gps_velned_msg(0),
        time_last_barometer_msg(0),
        dt_s_(0.0f),
        last_update_s_(0.0f),
        init_gps_position(false),
        last_alt(0),
        last_vel{0.0f,0.0f,0.0f},
        gravity(config.gravity),
        ahrs(ahrs),
        state(state),
        gps(gps),
        barometer(barometer),
        sonar(sonar)
{
    // default GPS home position
    origin_ =  config.origin;

    for(uint8_t i = 0; i < 3; i++)
    {
        local_position[i] = 0.0f;
        kp_pos_gps[i] = config.kp_pos_gps[i];
        kp_vel_gps[i] = config.kp_vel_gps[i];
    }
}


bool Position_estimation::update(void)
{
    // Updat timing
    float now      = time_keeper_get_s();
    dt_s_          = now - last_update_s_;
    last_update_s_ = now;

    if (ahrs.internal_state == AHRS_READY)
    {
        if (state.reset_position)
        {
            state.reset_position = false;
            reset_home_position();
        }

        position_integration();
        position_correction();

        if (state.is_armed())
        {
            fence_control();
        }

        return true;
    }
    else
    {
        return false;
    }
}


void Position_estimation::reset_home_position()
{
    // reset origin to position where quad is armed if we have GPS
    if (init_gps_position)
    {
        origin_  = gps.position_gf();
        last_gps_pos           = local_position;
    }
    //else
    //{
    //origin_.longitude = HOME_LONGITUDE;
    //origin_.latitude = HOME_LATITUDE;
    //origin_.altitude = HOME_ALTITUDE;
    //}

    // Correct barometer bias
    if (init_gps_position)
    {
        float current_altitude_gf = - local_position[Z]
                                    + origin_.altitude;
        barometer.calibrate_bias(current_altitude_gf);

        print_util_dbg_print("Offset of the barometer set to the GPS altitude, new altitude of:");
        print_util_dbg_print_num(barometer.altitude_gf(), 10);
        print_util_dbg_print(" ( ");
        print_util_dbg_print_num(local_position[2], 10);
        print_util_dbg_print("  ");
        print_util_dbg_print_num(origin_.altitude, 10);
        print_util_dbg_print(" )\r\n");
    }

    // reset position estimator
    last_alt = 0;
    for (int32_t i = 0; i < 3; i++)
    {
        last_vel[i] = 0.0f;
        local_position[i] = 0.0f;
        vel[i] = 0.0f;
        vel_bf[i] = 0.0f;
    }
}


/* THIS IS EVIL */
bool Position_estimation::set_home_position_global(global_position_t new_home_pos)
{
    bool result = false;

    if (!state.is_armed())
    {
        //coord_conventions_change_origin(&local_position, new_home_pos);
        origin_ = new_home_pos;

        // Set new home position from msg
        print_util_dbg_print("[POSITION ESTIMATION] Set new home location: ");
        print_util_dbg_print_num(origin_.latitude * 10000000.0f, 10);
        print_util_dbg_print(", ");
        print_util_dbg_print_num(origin_.longitude * 10000000.0f, 10);
        print_util_dbg_print(", ");
        print_util_dbg_print_num(origin_.altitude * 1000.0f, 10);
        print_util_dbg_print(")\r\n");

        result = true;
    }

    return result;
}


bool Position_estimation::set_home_to_current_position()
{
   print_util_dbg_print("[POSITION ESTIMATION] Set new home to current location: \r\n");
   global_position_t new_origin;
   coord_conventions_local_to_global_position(local_position, origin_, new_origin);
   return set_home_position_global(new_origin);
}


Position_estimation::fence_violation_state_t Position_estimation::get_fence_violation_state() const
{
    fence_violation_state_t fence_violation_state = IN_FENCE;
    if(state.out_of_fence_2)
    {
        fence_violation_state = OUTSIDE_FENCE2;
    }
    else if(state.out_of_fence_1)
    {
        fence_violation_state = OUTSIDE_FENCE1;
    }
    return fence_violation_state;
}


Position_estimation::conf_t Position_estimation::default_config()
{
    conf_t conf = {};

    // default home location (EFPL Esplanade)
    conf.origin        = ORIGIN_EPFL;

    conf.gravity       = 9.81f;

    conf.kp_pos_gps[X] = 2.0f;
    conf.kp_pos_gps[Y] = 2.0f;
    conf.kp_pos_gps[Z] = 0.0f;

    conf.kp_vel_gps[X] = 1.0f;
    conf.kp_vel_gps[Y] = 1.0f;
    conf.kp_vel_gps[Z] = 3.0f;

    conf.kp_alt_baro = 2.0f;
    conf.kp_vel_baro = 0.5f;

    conf.kp_alt_sonar = 2.0f;
    conf.kp_vel_sonar = 2.0f;

    return conf;
};


float Position_estimation::last_update_s(void) const
{
    return last_update_s_;
}


std::array<float,3> Position_estimation::position_lf(void) const
{
    return local_position;
}


std::array<float,3> Position_estimation::velocity_lf(void) const
{
    return vel;
}


float Position_estimation::absolute_altitude(void) const
{
    return (origin_.altitude - local_position[Z]);
}


bool Position_estimation::is_healthy(INS::healthy_t type) const
{
    return true;
}
