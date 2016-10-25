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
 * \file control_command.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief Data structures to hold torque, thrust, rate, attitude and
 * velocity commands
 *
 ******************************************************************************/

#ifndef CONTROL_COMMAND_HPP_
#define CONTROL_COMMAND_HPP_


#include "util/coord_conventions.hpp"

extern "C"
{
#include "util/quaternions.h"
}

/**
 * \brief   Control command mode enum
 */
typedef enum
{
    TORQUE_COMMAND   = 0,
    RATE_COMMAND     = 1,
    ATTITUDE_COMMAND = 2,
    POSITION_COMMAND = 3,
    VELOCITY_COMMAND = 4
} control_command_mode_t;


/**
 * \brief   Velocity command structure
 */
typedef struct
{
    local_velocity_t xyz;           ///<    Velocity on X, Y and Z axis (NED frame)
    float heading;                  ///<    Heading (can be ignored for fixed wing)
} velocity_command_t;


/**
 * \brief   Position command structure
 */
typedef struct
{
    local_position_t xyz;       ///<    Position in NED frame
    float heading;              ///<    Heading (can be ignored for fixed wing)
} position_command_t;


/**
 * \brief   Attitude command structure
 */
typedef quat_t attitude_command_t;


/**
 * \brief   Rate command structure (expressed in body frame)
 */
// class rate_command_t: public std::array<float,3>{};
// typedef std::array<float,3> rate_command_t;
typedef struct
{
    std::array<float,3> xyz;
} rate_command_t;

/**
 * \brief   Torque command structure (expressed in body frame)
 */
// class torque_command_t: public std::array<float,3>{};
// typedef std::array<float,3> torque_command_t;
typedef struct
{
    std::array<float,3> xyz;
} torque_command_t;


/**
 * \brief   Thrust command type for vehicles with multi-lifting devices
 */
typedef struct
{
    std::array<float,3> xyz;
} thrust_command_t;


/**
 * \brief   End command type, contains nothing
 */
typedef struct {} empty_command_t;

/**
 * \brief   Global command structure
 */
typedef struct
{
    control_command_mode_t  mode;       ///< Control command mode
    position_command_t      position;   ///< Position command
    velocity_command_t      velocity;   ///< Velocity command
    attitude_command_t      attitude;   ///< Attitude command
    rate_command_t          rate;       ///< Rate command
    torque_command_t        torque;     ///< Torque command
    thrust_command_t        thrust3D;   ///< Thrust command
} command_t;

#endif // CONTROL_COMMAND_HPP_
