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


Velocity_controller_copter::Velocity_controller_copter( const ahrs_t& ahrs,
                                                        const INS& ins,
                                                        const velocity_command_t& velocity_command,
                                                        attitude_command_t& attitude_command,
                                                        thrust_command_t& thrust_command,
                                                        conf_t config):
    ahrs_(ahrs),
    ins_(ins),
    velocity_command_(velocity_command),
    attitude_command_(attitude_command),
    thrust_command_(thrust_command),
    control_frame_(config.control_frame),
    thrust_hover_point_(config.thrust_hover_point)
{
    // Init PID gains
    pid_controller_init(&pid_[X], &config.pid_config[X]);
    pid_controller_init(&pid_[Y], &config.pid_config[Y]);
    pid_controller_init(&pid_[Z], &config.pid_config[Z]);
}


bool Velocity_controller_copter::update(void)
{
    float velocity[3]         = {0.0f, 0.0f, 0.0f};
    float velocity_command[3] = {0.0f, 0.0f, 0.0f};
    float errors[3]           = {0.0f, 0.0f, 0.0f};
    float thrust_vector[3]    = {0.0f, 0.0f, 0.0f};

    // Get rotation quaternion from local frame to semilocal frame
    float rpy_semilocal[3] = {0.0f, 0.0f, coord_conventions_get_yaw(ahrs_.qe)};
    quat_t q_semilocal = coord_conventions_quaternion_from_rpy(rpy_semilocal);

    // Compute velocity command
    switch (control_frame_)
    {
        case VEL_CTRL_LOCAL:
            // Get the command velocity in local frame
            switch (velocity_command_.mode)
            {
                case VELOCITY_COMMAND_MODE_SEMI_LOCAL:
                    quaternions_rotate_vector(q_semilocal,
                                              velocity_command_.xyz,
                                              velocity_command);
                break;

                case VELOCITY_COMMAND_MODE_LOCAL:
                    velocity_command[X] = velocity_command_.xyz[X];
                    velocity_command[Y] = velocity_command_.xyz[Y];
                    velocity_command[Z] = velocity_command_.xyz[Z];
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
            switch (velocity_command_.mode)
            {
                case VELOCITY_COMMAND_MODE_SEMI_LOCAL:
                    velocity_command[X] = velocity_command_.xyz[X];
                    velocity_command[Y] = velocity_command_.xyz[Y];
                    velocity_command[Z] = velocity_command_.xyz[Z];
                break;

                case VELOCITY_COMMAND_MODE_LOCAL:
                    quaternions_rotate_vector(quaternions_inverse(q_semilocal),
                                              velocity_command_.xyz,
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
    std::array<float,3> vel = ins_.velocity_lf();
    switch (control_frame_)
    {
        case VEL_CTRL_SEMI_LOCAL:
            // Rotate velocity from local to semilocal
            quaternions_rotate_vector(quaternions_inverse(q_semilocal),
                                      velocity_command_.xyz,
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
    thrust_vector[X] = pid_controller_update(&pid_[X], errors[X]);                // should be multiplied by mass
    thrust_vector[Y] = pid_controller_update(&pid_[Y], errors[Y]);                // should be multiplied by mass
    thrust_vector[Z] = pid_controller_update(&pid_[Z], errors[Z]);                // should be multiplied by mass

    // Express error in semilocal frame
    switch (control_frame_)
    {
        case VEL_CTRL_SEMI_LOCAL:
        break;

        case VEL_CTRL_LOCAL:
            quaternions_rotate_vector(quaternions_inverse(q_semilocal), thrust_vector, thrust_vector);
        break;
    }

    // Map thrust dir to attitude
    attitude_command_.rpy[ROLL]  = maths_clip(thrust_vector[Y], 1);
    attitude_command_.rpy[PITCH] = - maths_clip(thrust_vector[X], 1);
    // attitude_command_.rpy[YAW]   = UNTOUCHED;

    aero_attitude_t attitude;
    attitude.rpy[ROLL]  = attitude_command_.rpy[ROLL];
    attitude.rpy[PITCH] = attitude_command_.rpy[PITCH];
    attitude.rpy[YAW]   = attitude_command_.rpy[YAW];
    attitude_command_.quat = coord_conventions_quaternion_from_aero(attitude);

    // Map PID output to thrust
    thrust_command_.thrust  = thrust_hover_point_ - thrust_vector[Z];

    return true;
}
