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
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief   Direct integration of the position with the IMU data
 *
 * \param   pos_est                 The pointer to the position estimation structure
 */
static void position_estimation_position_integration(position_estimation_t* pos_est);


/**
 * \brief   Position correction with the GPS and the barometer
 *
 * \param   pos_est                 The pointer to the position estimation structure
 */
static void position_estimation_position_correction(position_estimation_t* pos_est);


/**
 * \brief   Initialization of the position estimation from the GPS position
 *
 * \param   pos_est         The pointer to the position estimation structure
 * \param   gps             The pointer to the GPS structure
 *
 * \return  void
 */
static void gps_position_init(position_estimation_t* pos_est);

/**
 * \brief   Check if the robot is going further from the working radius, delimited by those fences
 *
 * \param   pos_est         The pointer to the position estimation structure
 *
 * \return  void
 */
static void position_estimation_fence_control(position_estimation_t* pos_est);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static void position_estimation_position_integration(position_estimation_t* pos_est)
{
    int32_t i;
    float dt = pos_est->ahrs->dt;

    quat_t qvel_bf, qvel;

    qvel.s = 0;
    for (i = 0; i < 3; i++)
    {
        qvel.v[i] = pos_est->vel[i];
    }
    qvel_bf = quaternions_global_to_local(pos_est->ahrs->qe, qvel);
    for (i = 0; i < 3; i++)
    {
        pos_est->vel_bf[i] = qvel_bf.v[i];
        pos_est->vel_bf[i] = pos_est->vel_bf[i] * (1.0f - (VEL_DECAY * dt)) + pos_est->ahrs->linear_acc[i]  * dt;
    }

    // calculate velocity in global frame
    // vel = qe *vel_bf * qe - 1

    qvel_bf.s = 0.0f;
    qvel_bf.v[0] = pos_est->vel_bf[0];  // TODO: replace this by quat_rotate_vector()
    qvel_bf.v[1] = pos_est->vel_bf[1];
    qvel_bf.v[2] = pos_est->vel_bf[2];

    qvel = quaternions_local_to_global(pos_est->ahrs->qe, qvel_bf);

    pos_est->vel[0] = qvel.v[0];
    pos_est->vel[1] = qvel.v[1];
    pos_est->vel[2] = qvel.v[2];    // TODO: replace this by quat_rotate_vector()

    for (i = 0; i < 3; i++)
    {
        pos_est->local_position.pos[i] = pos_est->local_position.pos[i] * (1.0f - (POS_DECAY * dt)) + pos_est->vel[i] * dt;

        pos_est->local_position.heading = coord_conventions_get_yaw(pos_est->ahrs->qe);
    }

}


static void position_estimation_position_correction(position_estimation_t* pos_est)
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

    //we use ahrs->dt since it is updated at the same frequency as position_estimation
    float dt = pos_est->ahrs->dt;

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

    if (pos_est->init_barometer)
    {
        // altimeter correction
        if (pos_est->time_last_barometer_msg < pos_est->barometer->last_update_us())
        {
            pos_est->last_alt = - (pos_est->barometer->altitude_gf() - pos_est->local_position.origin.altitude);

            pos_est->time_last_barometer_msg = pos_est->barometer->last_update_us();
        }

        baro_gain = 1.0f;

        baro_alt_error = pos_est->last_alt  - pos_est->local_position.pos[2];
        baro_vel_error = pos_est->barometer->vertical_speed_lf() - pos_est->vel[2];
    }
    else
    {
        // Correct barometer bias
        float current_altitude_gf = - pos_est->local_position.pos[Z]
                                    + pos_est->local_position.origin.altitude;
        pos_est->barometer->calibrate_bias(current_altitude_gf);
        pos_est->init_barometer = true;
    }

    if (pos_est->init_gps_position)
    {
        if (pos_est->gps->fix() == true)
        {
            if ((pos_est->time_last_gps_posllh_msg < pos_est->gps->last_position_update_us()))
            {
                global_gps_position = pos_est->gps->position_gf();
                local_coordinates   = coord_conventions_global_to_local_position(global_gps_position, pos_est->local_position.origin);

                // compute GPS velocity estimate
                gps_dt = (pos_est->gps->last_position_update_us() - pos_est->time_last_gps_posllh_msg) / 1000000.0f;
                if (gps_dt > 0.001f)
                {
                    for (i = 0; i < 3; i++)
                    {
                        pos_est->last_vel[i] = (local_coordinates.pos[i] - pos_est->last_gps_pos.pos[i]) / gps_dt;
                    }
                }
                else
                {
                    print_util_dbg_print("GPS dt is too small!\r\n");
                }

                pos_est->time_last_gps_posllh_msg = pos_est->gps->last_position_update_us();
                pos_est->last_gps_pos = local_coordinates;
            }

            if (pos_est->time_last_gps_velned_msg < pos_est->gps->last_velocity_update_us())
            {
                pos_est->time_last_gps_velned_msg = pos_est->gps->last_velocity_update_us();
            }

            // Enable gps correction
            gps_gain = 1.0f;

            // Compute position error and velocity error from gps
            for (i = 0; i < 3; i++)
            {
                pos_error[i] = pos_est->last_gps_pos.pos[i] - pos_est->local_position.pos[i];
                vel_error[i] = pos_est->gps->velocity_lf()[i] - pos_est->vel[i];
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
        gps_position_init(pos_est);
        for (i = 0; i < 3; i++)
        {
            pos_error[i] = 0.0f;
            vel_error[i] = 0.0f;
        }
        gps_gain = 0.0f;
    }

    if (pos_est->sonar->healthy())
    {
        sonar_gain      = 1.0f;
        sonar_alt_error = - pos_est->sonar->distance() - pos_est->local_position.pos[Z];
        sonar_vel_error = - pos_est->sonar->velocity() - pos_est->vel[Z];
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
        pos_est->local_position.pos[i] += pos_est->kp_pos_gps[i] * gps_gain * pos_error[i] * dt;
    }
    pos_est->local_position.pos[Z] += pos_est->kp_alt_baro * baro_gain * baro_alt_error * dt;
    pos_est->local_position.pos[Z] += pos_est->kp_alt_sonar * sonar_gain * sonar_alt_error * dt;

    // Apply error correction to velocity estimates
    for (i = 0; i < 3; i++)
    {
        pos_est->vel[i] += pos_est->kp_vel_gps[i] * gps_gain * vel_error[i] * dt;
    }
    pos_est->vel[Z] += pos_est->kp_vel_baro * baro_gain * baro_vel_error * dt;
    pos_est->vel[Z] += pos_est->kp_vel_sonar * sonar_gain * sonar_vel_error * dt;
}


static void gps_position_init(position_estimation_t* pos_est)
{
    if ((pos_est->init_gps_position == false) && (pos_est->gps->fix() == true))
    {
        if ((pos_est->time_last_gps_posllh_msg < pos_est->gps->last_position_update_us())
                && (pos_est->time_last_gps_velned_msg < pos_est->gps->last_velocity_update_us()))
        {
            pos_est->time_last_gps_posllh_msg = pos_est->gps->last_position_update_us();
            pos_est->time_last_gps_velned_msg = pos_est->gps->last_velocity_update_us();

            pos_est->init_gps_position = true;

            pos_est->local_position.origin  = pos_est->gps->position_gf();
            pos_est->last_gps_pos           = pos_est->local_position;

            pos_est->last_alt = 0;
            for (int32_t i = 0; i < 3; i++)
            {
                pos_est->last_vel[i] = 0.0f;
                pos_est->local_position.pos[i] = 0.0f;
                pos_est->vel[i] = 0.0f;
            }

            print_util_dbg_print("GPS position initialized!\r\n");
        }
    }
}

static void position_estimation_fence_control(position_estimation_t* pos_est)
{
    float dist_xy_sqr, dist_z_sqr;
    dist_xy_sqr = SQR(pos_est->local_position.pos[X] - pos_est->fence_position.pos[X]) + SQR(pos_est->local_position.pos[Y] - pos_est->fence_position.pos[Y]);
    dist_z_sqr = SQR(pos_est->local_position.pos[Z] - pos_est->fence_position.pos[Z]);

    if (dist_xy_sqr > SQR(pos_est->state->fence_2_xy))
    {
        pos_est->state->out_of_fence_2 = true;
    }
    else if (dist_z_sqr > SQR(pos_est->state->fence_2_z))
    {
        pos_est->state->out_of_fence_2 = true;
    }
    else if (dist_xy_sqr > SQR(pos_est->state->fence_1_xy))
    {
        pos_est->state->out_of_fence_1 = true;
    }
    else if (dist_z_sqr > SQR(pos_est->state->fence_1_z))
    {
        pos_est->state->out_of_fence_1 = true;
    }
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool position_estimation_init(position_estimation_t* pos_est, const position_estimation_conf_t config, State* state, Barometer* barometer, const Sonar* sonar, const Gps* gps, const ahrs_t* ahrs)
{
    bool init_success = true;

    //init dependencies
    pos_est->barometer = barometer;
    pos_est->sonar = sonar;
    pos_est->gps = gps;
    pos_est->ahrs = ahrs;
    pos_est->state = state;
    pos_est->nav_plan_active = &state->nav_plan_active;

    // default GPS home position
    pos_est->local_position.origin.longitude =  config.origin.longitude;
    pos_est->local_position.origin.latitude =   config.origin.latitude;
    pos_est->local_position.origin.altitude =   config.origin.altitude;
    pos_est->local_position.pos[X] = 0;
    pos_est->local_position.pos[Y] = 0;
    pos_est->local_position.pos[Z] = 0;

    pos_est->fence_set = false;
    if (config.fence_set)
    {
        position_estimation_set_new_fence_origin(pos_est);
    }

    // reset position estimator
    pos_est->last_alt = 0;
    for (int32_t i = 0; i < 3; i++)
    {
        pos_est->last_vel[i] = 0.0f;
        pos_est->vel[i] = 0.0f;
        pos_est->vel_bf[i] = 0.0f;
    }

    pos_est->gravity = config.gravity;

    pos_est->init_gps_position = false;
    pos_est->init_barometer = false;
    pos_est->time_last_gps_posllh_msg = 0;
    pos_est->time_last_gps_velned_msg = 0;
    pos_est->time_last_barometer_msg = 0;

    pos_est->kp_pos_gps[X] = 2.0f;
    pos_est->kp_pos_gps[Y] = 2.0f;
    pos_est->kp_pos_gps[Z] = 0.0f;

    pos_est->kp_vel_gps[X] = 1.0f;
    pos_est->kp_vel_gps[Y] = 1.0f;
    pos_est->kp_vel_gps[Z] = 3.0f;

    pos_est->kp_alt_baro = 2.0f;
    pos_est->kp_vel_baro = 0.5f;

    pos_est->kp_alt_sonar = 2.0f;
    pos_est->kp_vel_sonar = 2.0f;

    gps_position_init(pos_est);

    return init_success;
}


void position_estimation_reset_home_altitude(position_estimation_t* pos_est)
{
    // reset origin to position where quad is armed if we have GPS
    if (pos_est->init_gps_position)
    {
        pos_est->local_position.origin  = pos_est->gps->position_gf();
        pos_est->last_gps_pos           = pos_est->local_position;
    }
    //else
    //{
    //pos_est->local_position.origin.longitude = HOME_LONGITUDE;
    //pos_est->local_position.origin.latitude = HOME_LATITUDE;
    //pos_est->local_position.origin.altitude = HOME_ALTITUDE;
    //}

    // Correct barometer bias
    float current_altitude_gf = - pos_est->local_position.pos[Z]
                                + pos_est->local_position.origin.altitude;
    pos_est->barometer->calibrate_bias(current_altitude_gf);
    pos_est->init_barometer = true;

    print_util_dbg_print("Offset of the barometer set to the GPS altitude, new altitude of:");
    print_util_dbg_print_num(pos_est->barometer->altitude_gf(), 10);
    print_util_dbg_print(" ( ");
    print_util_dbg_print_num(pos_est->local_position.pos[2], 10);
    print_util_dbg_print("  ");
    print_util_dbg_print_num(pos_est->local_position.origin.altitude, 10);
    print_util_dbg_print(" )\r\n");

    // reset position estimator
    pos_est->last_alt = 0;
    for (int32_t i = 0; i < 3; i++)
    {
        pos_est->last_vel[i] = 0.0f;
        pos_est->local_position.pos[i] = 0.0f;
        pos_est->vel[i] = 0.0f;
        pos_est->vel_bf[i] = 0.0f;
    }
}


void position_estimation_update(position_estimation_t* pos_est)
{
    if (pos_est->ahrs->internal_state == AHRS_READY)
    {
        if (pos_est->state->reset_position)
        {
            pos_est->state->reset_position = false;
            position_estimation_reset_home_altitude(pos_est);

            position_estimation_set_new_fence_origin(pos_est);
        }

        position_estimation_position_integration(pos_est);
        position_estimation_position_correction(pos_est);
        if (mav_modes_is_armed(pos_est->state->mav_mode) && pos_est->fence_set)
        {
            position_estimation_fence_control(pos_est);
        }
    }
}

void position_estimation_set_new_fence_origin(position_estimation_t* pos_est)
{
    if (!pos_est->fence_set)
    {
        print_util_dbg_print("Setting new fence origin position.\r\n");

        pos_est->fence_set = true;
        pos_est->fence_position.origin = pos_est->local_position.origin;
    }
    pos_est->fence_position = coord_conventions_global_to_local_position(pos_est->fence_position.origin, pos_est->local_position.origin);
}