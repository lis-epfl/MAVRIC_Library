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
 * \file velocity_controller_copter.cpp
 *
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Nicolas Dousse
 * \author Julien Lecoeur
 *
 * \brief A velocity controller for copters.
 *
 ******************************************************************************/

#include <array>

#include "control/velocity_controller_copter.hpp"
#include "util/coord_conventions.hpp"
#include "util/constants.hpp"

extern "C"
{
#include "util/quaternions.h"
#include "util/maths.h"
#include "util/vectors.h"
}


bool velocity_controller_copter_init(velocity_controller_copter_t* controller, velocity_controller_copter_conf_t config, const ahrs_t* ahrs, const INS* ins, const velocity_command_t* velocity_command, attitude_command_t* attitude_command, thrust_command_t* thrust_command)
{
    // Init dependencies
    controller->velocity_command    = velocity_command;
    controller->attitude_command    = attitude_command;
    controller->thrust_command      = thrust_command;
    controller->ahrs                = ahrs;
    controller->ins                 = ins;

    // Init hover point
    controller->thrust_hover_point = config.thrust_hover_point;
    controller->control_frame      = config.control_frame;

    // Init PID gains
    pid_controller_init(&controller->pid[X], &config.pid_config[X]);
    pid_controller_init(&controller->pid[Y], &config.pid_config[Y]);
    pid_controller_init(&controller->pid[Z], &config.pid_config[Z]);

    return true;
}


bool velocity_controller_copter_update(velocity_controller_copter_t* controller)
{
    float velocity[3];
    float velocity_command[3];
    float errors[3];
    float thrust_vector[3];

    // Get rotation quaternion from local frame to semilocal frame
    float rpy_semilocal[3] = {0.0f, 0.0f, coord_conventions_get_yaw(controller->ahrs->qe)};
    quat_t q_semilocal = coord_conventions_quaternion_from_rpy(rpy_semilocal);

    // Compute velocity command
    switch (controller->control_frame)
    {
        case VEL_CTRL_LOCAL:
            // Get the command velocity in local frame
            switch (controller->velocity_command->mode)
            {
                case VELOCITY_COMMAND_MODE_SEMI_LOCAL:
                    quaternions_rotate_vector(q_semilocal,
                                              controller->velocity_command->xyz,
                                              velocity_command);
                break;

                case VELOCITY_COMMAND_MODE_LOCAL:
                    velocity_command[X] = controller->velocity_command->xyz[X];
                    velocity_command[Y] = controller->velocity_command->xyz[Y];
                    velocity_command[Z] = controller->velocity_command->xyz[Z];
                break;

                default:
                    velocity_command[X] = 0.0f;
                    velocity_command[Y] = 0.0f;
                    velocity_command[Z] = 0.0f;
                break;
            }
        break;

        case VEL_CTRL_SEMI_LOCAL:
            // Get the command velocity in semi local frame
            switch (controller->velocity_command->mode)
            {
                case VELOCITY_COMMAND_MODE_SEMI_LOCAL:
                    velocity_command[X] = controller->velocity_command->xyz[X];
                    velocity_command[Y] = controller->velocity_command->xyz[Y];
                    velocity_command[Z] = controller->velocity_command->xyz[Z];
                break;

                case VELOCITY_COMMAND_MODE_LOCAL:
                    quaternions_rotate_vector(quaternions_inverse(q_semilocal),
                                              controller->velocity_command->xyz,
                                              velocity_command);
                break;

                default:
                    velocity_command[X] = 0.0f;
                    velocity_command[Y] = 0.0f;
                    velocity_command[Z] = 0.0f;
                break;
            }
        break;
    }

    // Compute current velocity
    std::array<float,3> vel = controller->ins->velocity_lf();
    switch (controller->control_frame)
    {
        case VEL_CTRL_SEMI_LOCAL:
            // Rotate velocity from local to semilocal
            quaternions_rotate_vector(quaternions_inverse(q_semilocal),
                                      controller->velocity_command->xyz,
                                      velocity);
        break;

        case VEL_CTRL_LOCAL:
            velocity[X] = vel[X];
            velocity[Y] = vel[Y];
            velocity[Z] = vel[Z];
        break;
    }

    // Compute errors in local NED frame
    errors[X] = velocity_command[X] - vel[X];
    errors[Y] = velocity_command[Y] - vel[Y];
    errors[Z] = velocity_command[Z] - vel[Z];       // WARNING: it was multiplied by (-1) in stabilisation_copter.c

    // Update PID
    thrust_vector[X] = pid_controller_update(&controller->pid[X], errors[X]);                // should be multiplied by mass
    thrust_vector[Y] = pid_controller_update(&controller->pid[Y], errors[Y]);                // should be multiplied by mass
    thrust_vector[Z] = pid_controller_update(&controller->pid[Z], errors[Z]);                // should be multiplied by mass

    // Express error in semilocal frame
    switch (controller->control_frame)
    {
        case VEL_CTRL_SEMI_LOCAL:
        break;

        case VEL_CTRL_LOCAL:
            quaternions_rotate_vector(quaternions_inverse(q_semilocal), thrust_vector, thrust_vector);
        break;
    }

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
