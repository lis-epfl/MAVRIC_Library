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
 * \file velocity_controller_fixedwing.cpp
 *
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Nicolas Dousse
 * \author Julien Lecoeur
 *
 * \brief A velocity controller for fixed wing platforms
 *
 ******************************************************************************/

#include <array>

#include "control/velocity_controller_fixedwing.hpp"
#include "util/coord_conventions.hpp"
#include "util/constants.hpp"
#include "util/quick_trig.hpp"

extern "C"
{
#include "util/quaternions.h"
#include "util/maths.h"
#include "util/vectors.h"
}


Velocity_controller_fixedwing::Velocity_controller_fixedwing(const args_t& args, const conf_t& config) :
    Velocity_controller(args.vel_args, config)
    // thrust_hover_point_(config.thrust_hover_point)
{}


bool Velocity_controller_fixedwing::compute_attitude_and_thrust_from_desired_accel(const std::array<float,3>& accel_vector,
                                                                                attitude_command_t& attitude_command,
                                                                                thrust_command_t& thrust_command)
{
    // Convert accel vector to semilocal frame
    quat_t q_semilocalframe = coord_conventions_quaternion_from_rpy(0.0f, 0.0f, ahrs_.yaw());
    std::array<float,3> accel_slf;
    quaternions_rotate_vector(quaternions_inverse(q_semilocalframe), accel_vector.data(), accel_slf.data());

    // Naive implementation TODO check this
    float roll   = accel_slf[Y];
    float pitch  = - accel_slf[Z];
    float yaw    = ahrs_.yaw();
    float thrust = accel_slf[X];

    // Output
    attitude_command      = coord_conventions_quaternion_from_rpy(roll, pitch, yaw);
    thrust_command.xyz[X] = thrust;
    thrust_command.xyz[Y] = 0.0f;
    thrust_command.xyz[Z] = 0.0f;

    return true;
}
