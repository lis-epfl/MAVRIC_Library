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
 * \file velocity_controller_copter.c
 *
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Nicolas Dousse
 * \author Julien Lecoeur
 *
 * \brief A velocity controller for copters.
 *
 ******************************************************************************/

#include "control/velocity_controller_copter.hpp"

extern "C"
{
#include "util/quaternions.h"
#include "util/coord_conventions.h"
#include "util/maths.h"
#include "util/constants.h"
#include "util/vectors.h"
}

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief Converts velocity command from body frame to local frame
 *
 * \param controller    Pointer to data structure
 * \param command[3]    Velocity command in local frame (output)
 */
static void get_velocity_command_from_body_to_local(const velocity_controller_copter_t* controller, float command[3]);


/**
 * \brief Converts velocity command from semi-local frame to local frame
 *
 * \details Semi local frame is global rotated around the vertical axis to match
 * the X axis with the current heading of the UAV
 *
 * \param controller    Pointer to data structure
 * \param command[3]    Velocity command in local frame (output)
 *
 */
static void get_velocity_command_from_semilocal_to_local(const velocity_controller_copter_t* controller, float command[3]);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static void get_velocity_command_from_body_to_local(const velocity_controller_copter_t* controller, float command[3])
{
    quaternions_rotate_vector(quaternions_inverse(controller->ahrs->qe),
                              controller->velocity_command->xyz,
                              command);
}


static void get_velocity_command_from_semilocal_to_local(const velocity_controller_copter_t* controller, float command[3])
{
    aero_attitude_t semilocal_frame_rotation;
    quat_t q_semilocal;

    // Get current heading
    float heading = coord_conventions_quat_to_aero(controller->ahrs->qe).rpy[YAW];
    semilocal_frame_rotation.rpy[ROLL]  = 0.0f;
    semilocal_frame_rotation.rpy[PITCH] = 0.0f;
    semilocal_frame_rotation.rpy[YAW]   = heading;

    // Get rotation quaternion from semilocal frame to local frame
    q_semilocal = coord_conventions_quaternion_from_aero(semilocal_frame_rotation);

    // Rotate command from semilocal to local
    quaternions_rotate_vector(q_semilocal,
                              controller->velocity_command->xyz,
                              command);
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool velocity_controller_copter_init(velocity_controller_copter_t* controller, velocity_controller_copter_conf_t config, const ahrs_t* ahrs, const Position_estimation* pos_est, const velocity_command_t* velocity_command, attitude_command_t* attitude_command, thrust_command_t* thrust_command)
{
    // Init dependencies
    controller->velocity_command    = velocity_command;
    controller->attitude_command    = attitude_command;
    controller->thrust_command      = thrust_command;
    controller->ahrs                = ahrs;
    controller->pos_est             = pos_est;

    // Init hover point
    controller->thrust_hover_point = config.thrust_hover_point;

    // Init PID gains
    pid_controller_init(&controller->pid[X], &config.pid_config[X]);
    pid_controller_init(&controller->pid[Y], &config.pid_config[Y]);
    pid_controller_init(&controller->pid[Z], &config.pid_config[Z]);

    return true;
}


bool velocity_controller_copter_update(velocity_controller_copter_t* controller)
{
    float velocity_command_local[3];
    float errors[3];
    float thrust_vector[3];

    // Get the command velocity in global frame
    switch (controller->velocity_command->mode)
    {
        case VELOCITY_COMMAND_MODE_BODY:
            get_velocity_command_from_body_to_local(controller,
                    velocity_command_local);
            break;

        case VELOCITY_COMMAND_MODE_SEMI_LOCAL:
            get_velocity_command_from_semilocal_to_local(controller,
                    velocity_command_local);
            break;

        case VELOCITY_COMMAND_MODE_LOCAL:
            velocity_command_local[X] = controller->velocity_command->xyz[X];
            velocity_command_local[Y] = controller->velocity_command->xyz[Y];
            velocity_command_local[Z] = controller->velocity_command->xyz[Z];
            break;

        default:
            velocity_command_local[X] = 0.0f;
            velocity_command_local[Y] = 0.0f;
            velocity_command_local[Z] = 0.0f;
            break;
    }

    // Compute errors in local NED frame
    errors[X] = velocity_command_local[X] - controller->pos_est->vel[X];
    errors[Y] = velocity_command_local[Y] - controller->pos_est->vel[Y];
    errors[Z] = velocity_command_local[Z] - controller->pos_est->vel[Z];       // WARNING: it was multiplied by (-1) in stabilisation_copter.c

    // Update PID in local frame
    thrust_vector[X] = pid_controller_update(&controller->pid[X], errors[X]);                // should be multiplied by mass
    thrust_vector[Y] = pid_controller_update(&controller->pid[Y], errors[Y]);                // should be multiplied by mass
    thrust_vector[Z] = pid_controller_update(&controller->pid[Z], errors[Z]);                // should be multiplied by mass

    // Rotate thrust vector to next semi_local frame
    aero_attitude_t attitude_yaw_inverse;
    attitude_yaw_inverse.rpy[0] = 0.0f;
    attitude_yaw_inverse.rpy[1] = 0.0f;
    attitude_yaw_inverse.rpy[2] = -controller->attitude_command->rpy[YAW];
    quat_t q_rot = coord_conventions_quaternion_from_aero(attitude_yaw_inverse);
    quaternions_rotate_vector(q_rot, thrust_vector, thrust_vector);

    // Map thrust dir to attitude
    controller->attitude_command->rpy[ROLL]  = maths_clip(thrust_vector[Y], 1);
    controller->attitude_command->rpy[PITCH] = - maths_clip(thrust_vector[X], 1);
    // controller->attitude_command->rpy[YAW]   = UNTOUCHED;

    aero_attitude_t attitude;
    attitude.rpy[ROLL]  = controller->attitude_command->rpy[ROLL];
    attitude.rpy[PITCH] = controller->attitude_command->rpy[PITCH];
    attitude.rpy[YAW]   = controller->attitude_command->rpy[YAW];
    controller->attitude_command->quat = coord_conventions_quaternion_from_aero(attitude);

    // Map PID output to thrust
    controller->thrust_command->thrust  = controller->thrust_hover_point - thrust_vector[Z];

    return true;
}
