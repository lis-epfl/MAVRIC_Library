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
 * \file velocity_controller_copter.hxx
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

template<class TAttitude_controller>
Velocity_controller_copter<TAttitude_controller>::Velocity_controller_copter( args_t args, conf_t config) :
    TAttitude_controller(args.attitude_controller_args, config.attitude_controller_config),
    ahrs_(args.ahrs),
    ins_(args.ins),
    thrust_hover_point_(config.thrust_hover_point)
{
    // Init PID gains
    pid_controller_init(&pid_[X], &config.pid_config[X]);
    pid_controller_init(&pid_[Y], &config.pid_config[Y]);
    pid_controller_init(&pid_[Z], &config.pid_config[Z]);

        /* set initial velocity command */
    vel_command_t initial_command;
    initial_command.vel = {0.0f,0.0f,0.0f};
    set_velocity_command(initial_command);
}


template<class TAttitude_controller>
void Velocity_controller_copter<TAttitude_controller>::update()
{
    /* check whether this is the highest active level of the cascade */
    if(TAttitude_controller::cascade_command_ == &velocity_command_)
    {
        /* calculate attitude_command and propagate down the cascade */
        typename TAttitude_controller::att_command_t attitude_command = calc_attitude_command(velocity_command_);
        TAttitude_controller::update_cascade(attitude_command);
    }else
    {
        /* propagate update() down the cascade until reaching the highest active level */
        TAttitude_controller::update();
    }
}


template<class TAttitude_controller>
bool Velocity_controller_copter<TAttitude_controller>::set_velocity_command(const vel_command_t& vel_command)
{
    velocity_command_ = vel_command;
    TAttitude_controller::cascade_command_ = &velocity_command_;
    return true;
}


//------------------------------------------------------------------------------
// PROTECTED FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

template<class TAttitude_controller>
void Velocity_controller_copter<TAttitude_controller>::update_cascade(const vel_command_t& vel_command)
{
    velocity_command_ = vel_command;
    typename TAttitude_controller::att_command_t attitude_command = calc_attitude_command(vel_command);
    TAttitude_controller::update_cascade(attitude_command);
}

template<class TAttitude_controller>
typename TAttitude_controller::att_command_t Velocity_controller_copter<TAttitude_controller>::calc_attitude_command(const vel_command_t& vel_command)
{
    // Get rotation quaternion from local frame to semilocal frame
    float rpy_semilocal[3] = {0.0f, 0.0f, coord_conventions_get_yaw(ahrs_.qe)};
    quat_t q_semilocal = coord_conventions_quaternion_from_rpy(rpy_semilocal);

    // Compute current velocity
    std::array<float,3> velocity = ins_.velocity_lf();

    // Compute errors in local NED frame
    float errors[3];
    errors[X] = velocity_command_.vel[X] - velocity[X];
    errors[Y] = velocity_command_.vel[Y] - velocity[Y];
    errors[Z] = velocity_command_.vel[Z] - velocity[Z];       // WARNING: it was multiplied by (-1) in stabilisation_copter.c

    // Update PID
    float thrust_vector[3];
    thrust_vector[X] = pid_controller_update(&pid_[X], errors[X]);                // should be multiplied by mass
    thrust_vector[Y] = pid_controller_update(&pid_[Y], errors[Y]);                // should be multiplied by mass
    thrust_vector[Z] = pid_controller_update(&pid_[Z], errors[Z]);                // should be multiplied by mass

    /* transform thrust_vector to semi-local frame */
    quaternions_rotate_vector(quaternions_inverse(q_semilocal), thrust_vector, thrust_vector);

    // TODO: calc desired yaw

    // Map thrust dir to attitude
    typename TAttitude_controller::att_command_t attitude_command;
    float rpy[3];
    rpy[ROLL]  = maths_clip(thrust_vector[Y], 1);
    rpy[PITCH] = - maths_clip(thrust_vector[X], 1);
    rpy[YAW]   = 0; //UNTOUCHED;
    attitude_command.att = coord_conventions_quaternion_from_rpy(rpy);

    // Map PID output to thrust
    attitude_command.thrust  = thrust_hover_point_ - thrust_vector[Z];

    return attitude_command;
}
