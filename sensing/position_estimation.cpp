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
 * \file position_estimation.c
 *
 * \author MAV'RIC Team
 *
 * \brief This file performs the 3D position estimation, either by direct
 * integration or with correction with the GPS and pos_est->barometer
 *
 ******************************************************************************/


#include "sensing/position_estimation.hpp"
#include "drivers/barometer.hpp"

extern "C"
{
#include "util/print_util.h"
#include "util/maths.h"
#include "hal/common/time_keeper.hpp"
}

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void Position_estimation::position_integration()
{
    int32_t i;
    float dt = ahrs.dt;

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
        local_position.pos[i] = local_position.pos[i] * (1.0f - (POS_DECAY * dt)) + vel[i] * dt;

        local_position.heading = coord_conventions_get_yaw(ahrs.qe);
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

    //we use ahrs.dt since it is updated at the same frequency as position_estimation
    float dt = ahrs.dt;

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

    if (init_barometer)
    {
        // altimeter correction
        if (time_last_barometer_msg < barometer.last_update_us())
        {
            last_alt = - (barometer.altitude_gf() - local_position.origin.altitude);

            time_last_barometer_msg = barometer.last_update_us();
        }

        baro_gain = 1.0f;

        baro_alt_error = last_alt  - local_position.pos[2];
        baro_vel_error = barometer.vertical_speed_lf() - vel[2];
    }
    else
    {
        // Correct barometer bias
        float current_altitude_gf = - local_position.pos[Z]
                                    + local_position.origin.altitude;
        barometer.calibrate_bias(current_altitude_gf);
        init_barometer = true;
    }

    if (init_gps_position)
    {
        if (gps.healthy() == true)
        {
            if ((time_last_gps_posllh_msg < gps.last_position_update_us()))
            {
                global_gps_position = gps.position_gf();
                local_coordinates   = coord_conventions_global_to_local_position(global_gps_position, local_position.origin);

                // compute GPS velocity estimate
                gps_dt = (gps.last_position_update_us() - time_last_gps_posllh_msg) / 1000000.0f;
                if (gps_dt > 0.001f)
                {
                    for (i = 0; i < 3; i++)
                    {
                        last_vel[i] = (local_coordinates.pos[i] - last_gps_pos.pos[i]) / gps_dt;
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
                pos_error[i] = last_gps_pos.pos[i] - local_position.pos[i];
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
        sonar_alt_error = - sonar.distance() - local_position.pos[Z];
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
        local_position.pos[i] += kp_pos_gps[i] * gps_gain * pos_error[i] * dt;
    }
    local_position.pos[Z] += kp_alt_baro * baro_gain * baro_alt_error * dt;
    local_position.pos[Z] += kp_alt_sonar * sonar_gain * sonar_alt_error * dt;

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

            local_position.origin  = gps.position_gf();
            last_gps_pos           = local_position;

            last_alt = 0;
            for (int32_t i = 0; i < 3; i++)
            {
                last_vel[i] = 0.0f;
                local_position.pos[i] = 0.0f;
                vel[i] = 0.0f;
            }

            print_util_dbg_print("GPS position initialized!\r\n");
        }
    }
}

void Position_estimation::fence_control()
{
    float dist_xy_sqr, dist_z_sqr;
    dist_xy_sqr = SQR(local_position.pos[X] - fence_position.pos[X]) + SQR(local_position.pos[Y] - fence_position.pos[Y]);
    dist_z_sqr = SQR(local_position.pos[Z] - fence_position.pos[Z]);

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


bool Position_estimation::healthy() const
{
    bool healthy = altitude_healthy();
    healthy &= init_gps_position;
    healthy &= gps.healthy();

    return healthy;

}


bool Position_estimation::altitude_healthy() const
{
    return init_barometer;
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Position_estimation::Position_estimation(State& state, Barometer& barometer, const Sonar& sonar, const Gps& gps, const ahrs_t& ahrs, const conf_t config) :
        vel{0.0f,0.0f,0.0f},
        vel_bf{0.0f,0.0f,0.0f},
        kp_alt_baro(config.kp_alt_baro),
        kp_vel_baro(config.kp_vel_baro),
        kp_alt_sonar(config.kp_alt_sonar),
        kp_vel_sonar(config.kp_vel_sonar),
        time_last_gps_posllh_msg(0),
        time_last_gps_velned_msg(0),
        time_last_barometer_msg(0),
        init_gps_position(false),
        init_barometer(false),
        last_alt(0),
        last_vel{0.0f,0.0f,0.0f},
        fence_set(config.fence_set),
        gravity(config.gravity),
        ahrs(ahrs),
        state(state),
        gps(gps),
        barometer(barometer),
        sonar(sonar)
{
    // default GPS home position
    local_position.origin.longitude =  config.origin.longitude;
    local_position.origin.latitude =   config.origin.latitude;
    local_position.origin.altitude =   config.origin.altitude;

    for(uint8_t i = 0; i < 3; i++)
    {
        local_position.pos[i] = 0.0f;
        kp_pos_gps[i] = config.kp_pos_gps[i];
        kp_vel_gps[i] = config.kp_vel_gps[i];
    }

    if (fence_set)
    {
        set_new_fence_origin();
    }

    gps_position_init();
}


void Position_estimation::update()
{
    if (ahrs.internal_state == AHRS_READY)
    {
        if (state.reset_position)
        {
            state.reset_position = false;
            reset_home_position();
            set_new_fence_origin();
        }

        position_integration();
        position_correction();
        if (state.is_armed() && fence_set)
        {
            fence_control();
        }
    }
}


void Position_estimation::reset_home_position()
{
    // reset origin to position where quad is armed if we have GPS
    if (init_gps_position)
    {
        local_position.origin  = gps.position_gf();
        last_gps_pos           = local_position;
    }
    //else
    //{
    //local_position.origin.longitude = HOME_LONGITUDE;
    //local_position.origin.latitude = HOME_LATITUDE;
    //local_position.origin.altitude = HOME_ALTITUDE;
    //}

    // Correct barometer bias
    float current_altitude_gf = - local_position.pos[Z]
                                + local_position.origin.altitude;
    barometer.calibrate_bias(current_altitude_gf);
    init_barometer = true;

    print_util_dbg_print("Offset of the barometer set to the GPS altitude, new altitude of:");
    print_util_dbg_print_num(barometer.altitude_gf(), 10);
    print_util_dbg_print(" ( ");
    print_util_dbg_print_num(local_position.pos[2], 10);
    print_util_dbg_print("  ");
    print_util_dbg_print_num(local_position.origin.altitude, 10);
    print_util_dbg_print(" )\r\n");

    // reset position estimator
    last_alt = 0;
    for (int32_t i = 0; i < 3; i++)
    {
        last_vel[i] = 0.0f;
        local_position.pos[i] = 0.0f;
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
        local_position.origin = new_home_pos;

        // Set new home position from msg
        print_util_dbg_print("[POSITION ESTIMATION] Set new home location: ");
        print_util_dbg_print_num(local_position.origin.latitude * 10000000.0f, 10);
        print_util_dbg_print(", ");
        print_util_dbg_print_num(local_position.origin.longitude * 10000000.0f, 10);
        print_util_dbg_print(", ");
        print_util_dbg_print_num(local_position.origin.altitude * 1000.0f, 10);
        print_util_dbg_print(")\r\n");

        /* is this wise? */
        set_new_fence_origin();

        state.nav_plan_active = false;

        result = true;
    }

    return result;
}


bool Position_estimation::set_home_to_current_position()
{
   print_util_dbg_print("[POSITION ESTIMATION] Set new home to current location: \r\n");
   return set_home_position_global(coord_conventions_local_to_global_position(local_position));
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


void Position_estimation::set_new_fence_origin()
{
    if (!fence_set)
    {
        print_util_dbg_print("Setting new fence origin position.\r\n");
        fence_set = true;
        fence_position.origin = local_position.origin;
    }
    fence_position = coord_conventions_global_to_local_position(fence_position.origin, local_position.origin);
}


Position_estimation::conf_t Position_estimation::default_config()
{
    conf_t conf = {};

    conf.origin                     = {};
    //default home location (EFPL Esplanade)
    conf.origin.longitude           = 6.566044801857777f;
    conf.origin.latitude            = 46.51852236174565f;
    conf.origin.altitude            = 400.0f;
    conf.gravity                    = 9.81f;
    conf.fence_set                  = false;

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
